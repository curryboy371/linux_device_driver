#define pr_fmt(fmt) "[VS1003_MYSPI] " fmt

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/minmax.h>

#include "my_spi.h"
#include "vs1003_def.h"

/*
    사용법

    // 볼륨 확인
    cat /sys/class/vs1003_class/vs1003/volume

    // 볼륨 설정 (percent)
    echo 75 | sudo tee /sys/class/vs1003_class/vs1003/volume

    // 스트리밍
    cat music.mp3 | sudo tee /dev/vs1003 > /dev/null
*/

/* Device Name */

#define DEV_NAME   "vs1003"
#define CLASS_NAME "vs1003_class"

/* Device Tree GPIO label names */
#define GPIO_LABEL_XCS  "myspi-xcs"
#define GPIO_LABEL_XDCS "myspi-xdcs"
#define GPIO_LABEL_DREQ "myspi-dreq"
#define GPIO_LABEL_XRST "myspi-xrst"

struct vs1003_data {
    my_spi_slave_data_t slave;
    struct mutex        spi_lock;   // spi 통신 동시 접근 제한

    /* GPIO */
    struct gpio_desc *dreq_gpio;
    struct gpio_desc *xcs_gpio;
    struct gpio_desc *xdcs_gpio;
    struct gpio_desc *xrst_gpio;

    /* char dev */
    struct class *class;
    struct cdev  cdev;
    dev_t        devt;
    struct device *device;

    struct mutex ctl_lock;          // vs1003 모듈 lock

    /* FIFO streaming */
    struct workqueue_struct *wq;
    struct kfifo fifo;
    struct mutex fifo_lock;
    wait_queue_head_t wq_space;
    struct work_struct xfer_work;
    int dreq_irq;
    bool xfer_enabled;

    /* volume */
    uint16_t volume_byte;         // 현재 적용된 볼륨 byte
    uint16_t volume_target_byte;  // 타겟 볼륨 byte
    atomic_t volume_pending;      // volume sci pending 여부

    /* state */
    atomic_t play_state;
};

static struct vs1003_data *vs1003_dev;

static int  vs1003_wait_dreq(struct vs1003_data *dev, int usec_timeout);
static int  vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data, bool bwait);
static int  vs1003_read_sci(struct vs1003_data *dev, uint8_t addr, uint16_t *out_val);
static int  vs1003_write_sdi(struct vs1003_data *dev, const uint8_t *buf, size_t len);

static bool vs1003_sci_work(struct vs1003_data* dev);
static bool vs1003_sdi_work(struct vs1003_data *dev, size_t budget);
static void vs1003_xfer_work(struct work_struct *work);

static int  vs1003_soft_reset(struct vs1003_data *dev);
static void vs1003_force_hard_reset(struct vs1003_data *dev);
static int  vs1003_init_chip(struct vs1003_data *dev);
static int  vs1003_init_kfifo(struct vs1003_data *dev);
static int  vs1003_kfifo_stop(struct vs1003_data *dev);
static void vs1003_kfifo_release(struct vs1003_data *dev);

/* --- GPIO helpers (XCS/XDCS active-low 가정 시 DT에서 active_low 플래그 활용 권장) --- */
static inline void xcs_select(struct vs1003_data *d)   { if (d->xcs_gpio)  gpiod_set_value(d->xcs_gpio, 1); }
static inline void xcs_deselect(struct vs1003_data *d) { if (d->xcs_gpio)  gpiod_set_value(d->xcs_gpio, 0); }
static inline void xdcs_select(struct vs1003_data *d)   { if (d->xdcs_gpio) gpiod_set_value(d->xdcs_gpio, 1); }
static inline void xdcs_deselect(struct vs1003_data *d) { if (d->xdcs_gpio) gpiod_set_value(d->xdcs_gpio, 0); }

/* --- sysfs: volume --- */
static ssize_t volume_percent_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct vs1003_data *vs = dev_get_drvdata(dev);
    uint8_t left_byte = (vs->volume_byte >> 8) & 0xFF;
    unsigned int percent = vs1003_vol_byte_to_percent(left_byte);
    pr_debug("volume_percent_show: raw=0x%04X, left=0x%02X, percent=%u\n",
             vs->volume_byte, left_byte, percent);
    return scnprintf(buf, PAGE_SIZE, "%u\n", percent);
}

static ssize_t volume_percent_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct vs1003_data *vs = dev_get_drvdata(dev);
    unsigned int percent = 0;
    int ret = kstrtouint(buf, 0, &percent);
    if (ret)
        return ret;

    percent = clamp(percent, 0u, 100u);

    /* 좌우 동일 적용 */
    {
        uint8_t vol_b = vs1003_vol_percent_to_byte(percent);
        uint16_t vol_reg = ((uint16_t)vol_b << 8) | vol_b;
        WRITE_ONCE(vs->volume_target_byte, vol_reg);
    }

    atomic_set(&vs->volume_pending, 1);
    queue_work(vs->wq, &vs->xfer_work);
    pr_debug("volume_percent_store: pending ...\n");
    return count;
}

static DEVICE_ATTR(volume, 0644, volume_percent_show, volume_percent_store);

static struct attribute *vs1003_attrs[] = {
    &dev_attr_volume.attr,
    NULL,
};
static const struct attribute_group vs1003_group = {
    .attrs = vs1003_attrs,
};

/* --- DREQ wait --- */
static int vs1003_wait_dreq(struct vs1003_data *dev, int usec_timeout)
{
    while (usec_timeout-- > 0) {
        int v = gpiod_get_value(dev->dreq_gpio);
        if (v)
            return 0;
        udelay(1);
    }
    pr_err("vs1003_wait_dreq: timeout, DREQ=%d\n", gpiod_get_value(dev->dreq_gpio));
    return -EIO;
}

/* --- my_spi wrapper (single xfer) --- */
static inline int vs1003_do_xfer(struct vs1003_data *dev, const void *tx, void *rx, size_t len)
{
    my_spi_transfer_t xfer = {
        .tx_buf = (void *)tx,
        .rx_buf = rx,
        .len    = len,
    };
    return my_spi_sync(&dev->slave, &xfer);
}

/* --- SCI write/read (XCS) --- */
static int vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data, bool bwait)
{
    int ret;
    uint8_t tx_buf[4] = {
        VS1003_WRITE_COMMAND, addr, (data >> 8) & 0xFF, data & 0xFF
    };

    pr_debug("vs1003_write_sci: addr=0x%02x, data=0x%04x\n", addr, data);

    if (bwait) {
        ret = vs1003_wait_dreq(dev, 100000);
        if (ret < 0) {
            pr_err("vs1003_write_sci: DREQ timeout before addr 0x%02x\n", addr);
            return ret;
        }
    } else {
        if (!gpiod_get_value(dev->dreq_gpio))
            return -EBUSY;
    }

    mutex_lock(&dev->spi_lock);
    xdcs_deselect(dev);  /* ensure SDI is idle */
    xcs_select(dev);
    udelay(1);

    ret = vs1003_do_xfer(dev, tx_buf, NULL, sizeof(tx_buf));

    udelay(1);
    xcs_deselect(dev);
    mutex_unlock(&dev->spi_lock);

    if (ret < 0) {
        pr_err("vs1003_write_sci: xfer failed, ret=%d\n", ret);
        return ret;
    }

    if (bwait) {
        ret = vs1003_wait_dreq(dev, 100000);
        if (ret < 0) {
            pr_warn("vs1003_write_sci: DREQ not HIGH after addr 0x%02x\n", addr);
            return ret;
        }
    } else {
        if (!gpiod_get_value(dev->dreq_gpio))
            return -EBUSY;
    }
    return 0;
}

static int vs1003_read_sci(struct vs1003_data *dev, uint8_t addr, uint16_t *out_val)
{
    int ret;
    uint8_t tx_buf[4] = { VS1003_READ_COMMAND, addr, 0xFF, 0xFF };
    uint8_t rx_buf[4] = { 0 };

    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("vs1003_read_sci: DREQ timeout before read\n");
        return ret;
    }

    mutex_lock(&dev->spi_lock);
    xdcs_deselect(dev);
    xcs_select(dev);
    udelay(1);

    ret = vs1003_do_xfer(dev, tx_buf, rx_buf, sizeof(tx_buf));

    udelay(1);
    xcs_deselect(dev);
    mutex_unlock(&dev->spi_lock);

    if (ret < 0) {
        pr_err("vs1003_read_sci: xfer failed\n");
        return ret;
    }

    if (out_val)
        *out_val = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];

    pr_debug("vs1003_read_sci: addr=0x%02x, val=0x%04x\n", addr, out_val ? *out_val : 0);
    return 0;
}

/* --- SDI write (XDCS) --- */
static int vs1003_write_sdi(struct vs1003_data *dev, const uint8_t *buf, size_t len)
{
    int ret;

    if (!len) return 0;

    ret = vs1003_wait_dreq(dev, 200000);
    if (ret < 0) return ret;

    mutex_lock(&dev->spi_lock);
    xcs_deselect(dev);
    xdcs_select(dev);

    ret = vs1003_do_xfer(dev, buf, NULL, len);

    xdcs_deselect(dev);
    mutex_unlock(&dev->spi_lock);
    return ret;
}

/* --- xfer work --- */
static bool vs1003_sci_work(struct vs1003_data* dev)
{
    if (!atomic_xchg(&dev->volume_pending, 0))
        return false;

    if (!gpiod_get_value(dev->dreq_gpio)) {
        atomic_set(&dev->volume_pending, 1);
        return false;
    }

    {
        const uint16_t target = READ_ONCE(dev->volume_target_byte);
        int ret = vs1003_write_sci(dev, SCI_VOL, target, false);
        if (ret == 0) {
            WRITE_ONCE(dev->volume_byte, target);
            pr_debug("sci_work: SCI_VOL=0x%04x applied\n", target);
            return true;
        }
        atomic_set(&dev->volume_pending, 1);
        pr_warn("sci_work: SCI_VOL apply failed ret=%d\n", ret);
    }
    return false;
}

static bool vs1003_sdi_work(struct vs1003_data *dev, size_t budget)
{
    int ret;
    uint8_t chunk[VS1003_CHUNK_SIZE];

    if (!gpiod_get_value(dev->dreq_gpio))
        return false;
    if (kfifo_is_empty(&dev->fifo))
        return false;

    {
        unsigned int copied_len =
            kfifo_out(&dev->fifo, chunk, min_t(size_t, sizeof(chunk), budget));
        if (copied_len == 0)
            return false;

        ret = vs1003_write_sdi(dev, chunk, copied_len);
        if (ret < 0) {
            pr_err("sdi_work: xfer failed ret=%d\n", ret);
            return false;
        }
    }
    return true;
}

static void vs1003_xfer_work(struct work_struct *work)
{
    struct vs1003_data *dev = container_of(work, struct vs1003_data, xfer_work);

    while (READ_ONCE(dev->xfer_enabled)) {
        bool did_any = false;

        /* 1) pending SCI (volume 등) 우선 처리 */
        if (vs1003_sci_work(dev))
            did_any = true;

        /* 2) SDI 전송 */
        if (!kfifo_is_empty(&dev->fifo)) {
            for (int i = 0; i < 4; ++i) {
                if (!vs1003_sdi_work(dev, VS1003_CHUNK_SIZE))
                    break;
                did_any = true;

                /* FIFO 공간 알림 */
                wake_up_interruptible(&dev->wq_space);

                if (atomic_read(&dev->play_state) != PLAY_PLAYING)
                    atomic_set(&dev->play_state, PLAY_PLAYING);

                if (atomic_read(&dev->volume_pending))
                    break;
            }
        }

        if (!did_any) {
            if (kfifo_is_empty(&dev->fifo) &&
                atomic_read(&dev->play_state) != PLAY_STOPPED) {
                atomic_set(&dev->play_state, PLAY_STOPPED);
                pr_debug("xfer_work: state -> PLAY_STOPPED\n");
            }
            break;
        }

        if (atomic_read(&dev->volume_pending)) {
            cond_resched();
            continue;
        }
        cond_resched();
    }
}

/* --- DREQ IRQ --- */
static irqreturn_t vs1003_dreq_irq_thread(int irq, void *data)
{
    struct vs1003_data *dev = data;

    if (likely(gpiod_get_value(dev->dreq_gpio))) {
        if (READ_ONCE(dev->xfer_enabled) &&
            (!kfifo_is_empty(&dev->fifo) ||
             atomic_read(&dev->volume_pending) ||
             atomic_read(&dev->play_state) == PLAY_PLAYING)) {
            queue_work(dev->wq, &dev->xfer_work);
        }
    }
    return IRQ_HANDLED;
}

/* --- Reset / Init --- */
static void vs1003_force_hard_reset(struct vs1003_data *dev)
{
    pr_debug("vs1003_force_hard_reset\n");

    xcs_deselect(dev);
    xdcs_deselect(dev);

    if (dev->xrst_gpio) {
        gpiod_set_value(dev->xrst_gpio, 0);
        msleep(10);
        gpiod_set_value(dev->xrst_gpio, 1);
        msleep(100);
    } else {
        /* XRST 없으면 최소 대기 */
        msleep(100);
    }
}

static int vs1003_soft_reset(struct vs1003_data *dev)
{
    int ret;
    uint16_t val;

    pr_debug("vs1003_soft_reset\n");

    ret = vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW, true);
    if (ret < 0) {
        pr_err("soft_reset: write SCI_MODE failed\n");
        return ret;
    }

    msleep(2);

    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("soft_reset: DREQ timeout\n");
        return ret;
    }

    ret = vs1003_write_sci(dev, SCI_CLOCKF, VS1003_DEFAULT_CLOCKF, true);
    if (ret < 0) return ret;
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_CLOCKF, &val);
    if (ret < 0 || val != VS1003_DEFAULT_CLOCKF)
        pr_warn("SCI_CLOCKF mismatch: exp=0x%04x got=0x%04x\n",
                VS1003_DEFAULT_CLOCKF, val);

    ret = vs1003_write_sci(dev, SCI_AUDATA, VS1003_DEFAULT_AUDATA, true);
    if (ret < 0) return ret;
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_AUDATA, &val);
    if (ret < 0 || val != VS1003_DEFAULT_AUDATA)
        pr_warn("SCI_AUDATA mismatch: exp=0x%04x got=0x%04x\n",
                VS1003_DEFAULT_AUDATA, val);

    ret = vs1003_write_sci(dev, SCI_VOL, dev->volume_byte, true);
    if (ret < 0) return ret;
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_VOL, &val);
    if (ret < 0 || val != dev->volume_byte)
        pr_warn("SCI_VOL mismatch: exp=0x%04x got=0x%04x\n",
                dev->volume_byte, val);

    return 0;
}

static int vs1003_init_chip(struct vs1003_data *dev)
{
    int ret;

    pr_info("vs1003_init_chip\n");

    vs1003_force_hard_reset(dev);

    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("init_chip: DREQ timeout after hard reset\n");
        return ret;
    }

    {
        uint8_t vb = vs1003_vol_percent_to_byte(VS1003_DEFAULT_VOLUME_PER);
        uint16_t vol = ((uint16_t)vb << 8) | vb;
        dev->volume_byte = vol;
        atomic_set(&dev->volume_pending, 0);
    }

    return vs1003_soft_reset(dev);
}

/* --- Self test (Sine) --- */
static int vs1003_diag_sine_start(struct vs1003_data *dev, uint8_t freq_code)
{
    int ret;
    uint16_t mode;

    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;

    ret = vs1003_write_sci(dev, SCI_MODE, mode | SM_TESTS, true);
    if (ret < 0) return ret;

    {
        uint8_t seq[8] = { 0x53, 0xEF, 0x6E, freq_code, 0,0,0,0 };
        ret = vs1003_write_sdi(dev, seq, sizeof(seq));
        if (ret < 0) return ret;
    }

    pr_info("DIAG SINE start code=0x%02X\n", freq_code);
    return 0;
}

static int vs1003_diag_sine_stop(struct vs1003_data *dev)
{
    int ret;
    uint16_t mode;

    {
        uint8_t seq[8] = { 0x45, 0x78, 0x69, 0x74, 0,0,0,0 };
        ret = vs1003_write_sdi(dev, seq, sizeof(seq));
        if (ret < 0) return ret;
    }

    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;
    ret = vs1003_write_sci(dev, SCI_MODE, mode & ~SM_TESTS, true);
    if (ret < 0) return ret;

    pr_info("DIAG SINE stop\n");
    return 0;
}

static int vs1003_diag_selftest(struct vs1003_data *dev)
{
    int ret = vs1003_diag_sine_start(dev, 0x24); /* 1kHz */
    if (ret < 0) return ret;
    msleep(2000);
    return vs1003_diag_sine_stop(dev);
}

/* --- KFIFO init/stop/release --- */
static int vs1003_init_kfifo(struct vs1003_data *dev)
{
    int ret;

    dev->dreq_irq = -EINVAL;
    mutex_init(&dev->fifo_lock);

    ret = kfifo_alloc(&dev->fifo, VS1003_FIFO_SIZE, GFP_KERNEL);
    if (ret) {
        pr_err("kfifo_alloc failed\n");
        return ret;
    }

    dev->wq = alloc_workqueue("vs1003_wq", WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
    if (!dev->wq) {
        pr_err("alloc_workqueue failed\n");
        ret = -ENOMEM;
        goto err_fifo;
    }

    init_waitqueue_head(&dev->wq_space);
    INIT_WORK(&dev->xfer_work, vs1003_xfer_work);

    /* DREQ IRQ */
    dev->dreq_irq = gpiod_to_irq(dev->dreq_gpio);
    if (dev->dreq_irq < 0) {
        pr_err("gpiod_to_irq(dreq) failed: %d\n", dev->dreq_irq);
        ret = dev->dreq_irq;
        goto err_wq;
    }

    ret = request_threaded_irq(dev->dreq_irq, NULL, vs1003_dreq_irq_thread,
                               IRQF_ONESHOT | IRQF_TRIGGER_RISING,
                               "vs1003-dreq", dev);
    if (ret) {
        pr_err("request_threaded_irq failed: %d\n", ret);
        goto err_wq;
    }

    dev->xfer_enabled = true;
    atomic_set(&dev->play_state, PLAY_STOPPED);

    if (gpiod_get_value(dev->dreq_gpio))
        queue_work(dev->wq, &dev->xfer_work);

    return 0;

err_wq:
    if (dev->wq) {
        flush_workqueue(dev->wq);
        destroy_workqueue(dev->wq);
        dev->wq = NULL;
    }
err_fifo:
    if (dev->dreq_irq >= 0) {
        free_irq(dev->dreq_irq, dev);
        dev->dreq_irq = -EINVAL;
    }
    kfifo_free(&dev->fifo);
    return ret;
}

static int vs1003_kfifo_stop(struct vs1003_data *dev)
{
    int ret;
    uint16_t mode;

    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;

    ret = vs1003_write_sci(dev, SCI_MODE, mode | SM_CANCEL, true);
    if (ret < 0) return ret;

    /* end-fill */
    {
        uint8_t fill[32] = {0};
        int remain = 8192;

        while (remain > 0) {
            ret = vs1003_wait_dreq(dev, 200000);
            if (ret < 0) {
                pr_err("kfifo_stop: DREQ timeout\n");
                break;
            }
            vs1003_write_sdi(dev, fill, sizeof(fill));
            udelay(1);
            remain -= sizeof(fill);
        }
    }

    /* SM_CANCEL clear 확인 */
    for (int i = 0; i < 200; i++) {
        ret = vs1003_read_sci(dev, SCI_MODE, &mode);
        if (ret < 0) break;
        if ((mode & SM_CANCEL) == 0)
            break;
        udelay(200);
    }

    atomic_set(&dev->play_state, PLAY_STOPPED);
    return 0;
}

static void vs1003_kfifo_release(struct vs1003_data *dev)
{
    WRITE_ONCE(dev->xfer_enabled, false);

    if (dev->dreq_irq >= 0) {
        synchronize_irq(dev->dreq_irq);
        free_irq(dev->dreq_irq, dev);
        dev->dreq_irq = -EINVAL;
    }

    flush_work(&dev->xfer_work);
    if (dev->wq) {
        flush_workqueue(dev->wq);
        destroy_workqueue(dev->wq);
        dev->wq = NULL;
    }

    kfifo_free(&dev->fifo);
}

/* --- file ops --- */
static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct vs1003_data *dev = vs1003_dev;
    ssize_t written = 0;

    if (count == 0)
        return 0;

    while (written < count) {
        size_t to_copy = min_t(size_t, count - written, (size_t)VS1003_MAX_BUFFER_SIZE);
        unsigned int copied = 0;

        if (wait_event_interruptible(dev->wq_space,
                                     kfifo_avail(&dev->fifo) > 0 ||
                                     !READ_ONCE(dev->xfer_enabled)))
            return -ERESTARTSYS;

        if (!dev->xfer_enabled)
            return -EPIPE;

        mutex_lock(&dev->fifo_lock);
        {
            int ret = kfifo_from_user(&dev->fifo, buf + written, to_copy, &copied);
            if (ret < 0) { mutex_unlock(&dev->fifo_lock); return ret; }
        }
        mutex_unlock(&dev->fifo_lock);

        written += copied;
        queue_work(dev->wq, &dev->xfer_work);

        if (copied == 0)
            cond_resched();
    }
    return written;
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&vs1003_dev->ctl_lock))
        return -EBUSY;

    if (vs1003_soft_reset(vs1003_dev) < 0) {
        pr_warn("open: soft reset failed, try hard reset\n");
        vs1003_force_hard_reset(vs1003_dev);
        if (vs1003_soft_reset(vs1003_dev) < 0) {
            pr_err("open: hard reset also failed\n");
            mutex_unlock(&vs1003_dev->ctl_lock);
            return -EIO;
        }
    }
    return 0;
}

static int vs1003_release(struct inode *inode, struct file *file)
{
    mutex_unlock(&vs1003_dev->ctl_lock);
    return 0;
}

static const struct file_operations vs1003_fops = {
    .owner   = THIS_MODULE,
    .open    = vs1003_open,
    .write   = vs1003_write,
    .release = vs1003_release,
};

/* --- module init/exit: my_spi 등록 + DT GPIO 획득 + char dev + sysfs --- */
static int __init vs1003_init(void)
{
    int ret = 0;
    struct device_node *np = NULL;

    vs1003_dev = kzalloc(sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev)
        return -ENOMEM;

    mutex_init(&vs1003_dev->ctl_lock);
    mutex_init(&vs1003_dev->spi_lock);

    /* my_spi init & register */
    my_spi_slave_init(&vs1003_dev->slave);
    vs1003_dev->slave.mode = MYSPI_MODE_0;
    ret = my_spi_register(&vs1003_dev->slave);
    if (ret != SPI_ERR_NONE) {
        pr_err("my_spi_register failed (%d)\n", ret);
        kfree(vs1003_dev);
        vs1003_dev = NULL;
        return -EIO;
    }


    np = of_find_compatible_node(NULL, NULL, "chan,my-spi");
    if (!np) {
        pr_warn("DT node 'chan,my-spi' not found. GPIO via of_get_named_gpio will be skipped.\n");
    } else {
        int gpio;

        gpio = of_get_named_gpio(np, GPIO_LABEL_XCS, 0);
        if (gpio_is_valid(gpio))
            vs1003_dev->xcs_gpio = gpio_to_desc(gpio);

        gpio = of_get_named_gpio(np, GPIO_LABEL_XDCS, 0);
        if (gpio_is_valid(gpio))
            vs1003_dev->xdcs_gpio = gpio_to_desc(gpio);

        gpio = of_get_named_gpio(np, GPIO_LABEL_DREQ, 0);
        if (gpio_is_valid(gpio)) 
            vs1003_dev->dreq_gpio = gpio_to_desc(gpio);

        gpio = of_get_named_gpio(np, GPIO_LABEL_XRST, 0);
        if (gpio_is_valid(gpio))
            vs1003_dev->xrst_gpio = gpio_to_desc(gpio);
    }

    if (!vs1003_dev->dreq_gpio) {
        pr_err("DREQ gpio is mandatory\n");
        ret = -EINVAL;
        goto err_mspi;
    }
    if (!vs1003_dev->xcs_gpio || !vs1003_dev->xdcs_gpio) {
        pr_err("XCS/XDCS gpios are mandatory for VS1003\n");
        ret = -EINVAL;
        goto err_mspi;
    }

    /* char dev 등록 */
    ret = alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("alloc_chrdev_region failed\n");
        goto err_mspi;
    }

    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    ret = cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);
    if (ret < 0) {
        pr_err("cdev_add failed\n");
        goto err_chrdev;
    }

    vs1003_dev->class = class_create(DEV_NAME);
    if (IS_ERR(vs1003_dev->class)) {
        pr_err("class_create failed\n");
        ret = PTR_ERR(vs1003_dev->class);
        vs1003_dev->class = NULL;
        goto err_cdev;
    }

    vs1003_dev->device = device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);
    if (IS_ERR(vs1003_dev->device)) {
        pr_err("device_create failed\n");
        ret = PTR_ERR(vs1003_dev->device);
        vs1003_dev->device = NULL;
        goto err_class;
    }

    dev_set_drvdata(vs1003_dev->device, vs1003_dev);
    ret = sysfs_create_group(&vs1003_dev->device->kobj, &vs1003_group);
    if (ret) {
        pr_err("sysfs_create_group failed: %d\n", ret);
        goto err_device;
    }

    /* 칩 초기화 */
    ret = vs1003_init_chip(vs1003_dev);
    if (ret < 0) {
        pr_err("chip init failed\n");
        goto err_sysfs;
    }

    /* 스트리밍 초기화 */
    ret = vs1003_init_kfifo(vs1003_dev);
    if (ret < 0) {
        pr_err("kfifo init failed\n");
        goto err_sysfs;
    }

    /* 간단한 자가 테스트 (원하면 주석처리 가능) */
    vs1003_diag_selftest(vs1003_dev);

    pr_info("VS1003 my_spi driver initialized\n");
    return 0;

err_sysfs:
    if (vs1003_dev->device)
        sysfs_remove_group(&vs1003_dev->device->kobj, &vs1003_group);
err_device:
    if (vs1003_dev->device)
        device_destroy(vs1003_dev->class, vs1003_dev->devt);
err_class:
    if (vs1003_dev->class)
        class_destroy(vs1003_dev->class);
err_cdev:
    cdev_del(&vs1003_dev->cdev);
err_chrdev:
    unregister_chrdev_region(vs1003_dev->devt, 1);
err_mspi:
    my_spi_unregister(&vs1003_dev->slave);
    if (vs1003_dev) {
        kfree(vs1003_dev);
        vs1003_dev = NULL;
    }
    return ret;
}

static void __exit vs1003_exit(void)
{
    pr_info("VS1003 my_spi driver exit\n");

    if (!vs1003_dev)
        return;

    /* 스트리밍 종료 */
    vs1003_kfifo_stop(vs1003_dev);
    vs1003_kfifo_release(vs1003_dev);

    /* sysfs / chardev */
    if (vs1003_dev->device)
        sysfs_remove_group(&vs1003_dev->device->kobj, &vs1003_group);

    if (vs1003_dev->device)
        device_destroy(vs1003_dev->class, vs1003_dev->devt);
    if (vs1003_dev->class)
        class_destroy(vs1003_dev->class);
    cdev_del(&vs1003_dev->cdev);
    unregister_chrdev_region(vs1003_dev->devt, 1);

    /* my_spi */
    my_spi_unregister(&vs1003_dev->slave);

    /* GPIO descs 는 of_get_named_gpio + gpio_to_desc 로 얻어와서 put 생략 가능
       (필요시 gpiod_put 호출 추가) */

    kfree(vs1003_dev);
    vs1003_dev = NULL;
}

module_init(vs1003_init);
module_exit(vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chan");
MODULE_DESCRIPTION("VS1003 Linux Character Driver using my_spi");
