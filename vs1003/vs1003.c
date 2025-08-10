#define pr_fmt(fmt) "[VS1003] " fmt

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/atomic.h>

#include "vs1003_def.h"

/*

    // 볼륨 확인
    cat /sys/class/vs1003_class/vs1003/volume

    // 볼륨 설정 (percent)
    echo 75 | sudo tee /sys/class/vs1003_class/vs1003/volume
*/


#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"

struct vs1003_data {
    struct spi_device *spi;
    struct gpio_desc* dreq_gpio;
    struct gpio_desc* xcs_gpio;
    struct gpio_desc* xdcs_gpio;
    struct gpio_desc* xrst_gpio;
    struct class *class;
    struct cdev cdev;
    dev_t devt;
    struct mutex lock;
    struct device* device;


    // KFIFO Streaming
    struct workqueue_struct *wq;
    struct kfifo fifo;
    struct mutex fifo_lock;          // FIFO 보호
    wait_queue_head_t wq_space;    // FIFO 공간 대기
    struct work_struct xfer_work;  // 전송 워커
    int dreq_irq;                  // DREQ IRQ 번호
    bool xfer_enabled;             // fifo 전송 가능 상태 플래그


    // volume 설정
    uint16_t volume_byte;          // 설정된 볼륨 byte 값
    uint16_t volume_target_byte;   // target 볼륨 byte 값
    atomic_t volume_pending;       // 1이면 pending

    atomic_t play_state;            // 재생 상태
};

static struct vs1003_data *vs1003_dev;


static inline uint8_t vs1003_vol_percent_to_byte(unsigned int percent);
static inline unsigned int vs1003_vol_byte_to_percent(uint8_t v);

static ssize_t volume_percent_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t volume_percent_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static int vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data);

static int vs1003_soft_reset(struct vs1003_data *dev);
static void vs1003_force_hard_reset(struct vs1003_data *dev);

// xcs와 xdcs는 dts에서 active low이므로 코드상 설정과 물리적 출력이 반대임
static inline void xcs_select(struct vs1003_data *d)   { gpiod_set_value(d->xcs_gpio, 1); }
static inline void xcs_deselect(struct vs1003_data *d) { gpiod_set_value(d->xcs_gpio, 0); }

static inline void xdcs_select(struct vs1003_data *d)   { gpiod_set_value(d->xdcs_gpio, 1); }
static inline void xdcs_deselect(struct vs1003_data *d) { gpiod_set_value(d->xdcs_gpio, 0); }



// attribute 구조체 정의
static struct device_attribute dev_attr_volume = {
    .attr = {
        .name = "volume",
        .mode  = 0644,
    },
    .show = volume_percent_show,
    .store = volume_percent_store,
};

// attribute array 생성
static struct attribute *vs1003_attrs[] = {
    &dev_attr_volume.attr,
    NULL,
};

// grroup화
static const struct attribute_group vs1003_group = {
    .attrs = vs1003_attrs,
};

/* volume_percent: 0~100 */
static ssize_t volume_percent_show(struct device* dev, struct device_attribute* attr, char* buf) {

    struct vs1003_data* vs1003_dev = dev_get_drvdata(dev);

    // 좌우가 동일하므로 왼쪽만 체크
    uint8_t left_byte = (vs1003_dev->volume_byte >> 8) & 0xFF;
    unsigned int percent = vs1003_vol_byte_to_percent(left_byte);

    pr_debug("volume_percent_show: raw=0x%04X, left=0x%02X, percent=%u\n", vs1003_dev->volume_byte, left_byte, percent);

    return scnprintf(buf, PAGE_SIZE, "%u\n", percent);
}

static ssize_t volume_percent_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {


    struct vs1003_data* vs1003_dev = dev_get_drvdata(dev);

    unsigned int percent = 0;
    int ret = kstrtouint(buf, 0, &percent);
    if (ret) {
        return ret;
    }

    percent = clamp(percent, 0u, 100u);

    uint8_t vol_byte = vs1003_vol_percent_to_byte(percent);
    uint16_t vol_reg = ((uint16_t)vol_byte << 8) | vol_byte;


    // 재생중인 경우 비동기로 적용함
    if (atomic_read(&vs1003_dev->play_state) == PLAY_PLAYING) {
        vs1003_dev->volume_target_byte = vol_reg;

        // 멀티 CPU 간 메모리 쓰기 순서만 보장 (SMP 환경에서만 의미 있음, UP에서는 no-op)
        smp_wmb();   

        atomic_set(&vs1003_dev->volume_pending, 1);
        queue_work(vs1003_dev->wq, &vs1003_dev->xfer_work);
        pr_debug("volume_percent_store: pending ...\n");
        return count;
    }

    // 재생중이지 않은 경우 바로 적용
    mutex_lock(&vs1003_dev->lock);
    ret = vs1003_write_sci(vs1003_dev, SCI_VOL, vol_reg);
    mutex_unlock(&vs1003_dev->lock);
    if (ret) {
        return ret;
    }
    pr_debug("volume_percent_store: set input=%u, clamped=%u, vol_byte=0x%02X, vol_reg=0x%04X\n", percent, percent, vol_byte, vol_reg);

    vs1003_dev->volume_byte = vol_reg;
    return count;
}

// percent to volume reg byte로 변환하는 함수
static inline uint8_t vs1003_vol_percent_to_byte(unsigned int percent) {
    if (percent >= 100) {
        return MAX_VOLUME;
    }

    return (uint8_t)((100 - percent) * MIN_VOLUME / 100);
}

// volume byte를 percent로 변환
static inline unsigned int vs1003_vol_byte_to_percent(uint8_t v) {
    
    if (v == MAX_VOLUME) return 100;
    if (v >= MIN_VOLUME) return 0;

    return (unsigned int)((MIN_VOLUME - v) * 100 / MIN_VOLUME);
}

static int vs1003_wait_dreq(struct vs1003_data *dev, int usec_timeout) {
    int dreq_val;
    while (usec_timeout-- > 0) {
        dreq_val = gpiod_get_value(dev->dreq_gpio);
        if (dreq_val)
            return 0;
        udelay(1);
    }
    // 타임아웃

    dreq_val = gpiod_get_value(dev->dreq_gpio);
    pr_err("vs1003_wait_dreq: timeout, DREQ=%d\n", dreq_val);
    return -EIO;
}

static irqreturn_t vs1003_dreq_irq_thread(int irq, void *data) {

    struct vs1003_data *dev = data;
    if (likely(gpiod_get_value(dev->dreq_gpio))) {
        queue_work(dev->wq, &dev->xfer_work); // 전용 WQ 사용
    }

    return IRQ_HANDLED;
}

static int vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data) {
    int ret;
    
    pr_debug("vs1003_write_sci: addr=0x%02x, data=0x%04x\n", addr, data);

    uint8_t tx_buf[4] = {
        VS1003_WRITE_COMMAND,  // 0x02
        addr,                  // 레지스터 주소
        (data >> 8) & 0xFF,    // 상위 바이트
        data & 0xFF            // 하위 바이트
    };

    // 전송 전 DREQ가 HIGH 확인
    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("vs1003_write_sci: DREQ timeout before writing to addr 0x%02x\n", addr);
        return ret;
    }

    xcs_select(dev);
    udelay(1);

    // SPI 전송 구조체 구성
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = sizeof(tx_buf),
        .speed_hz = VS1003_CMD_HZ,
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    // SPI 전송 수행
    ret = spi_sync(dev->spi, &m);
    
    // 전송 완료 후 지연
    udelay(1);
    
    xcs_deselect(dev);

    if (ret < 0) {
        pr_err("vs1003_write_sci: spi_sync failed for addr 0x%02x, ret=%d\n", addr, ret);
        return ret;
    }

    // 전송 후 DREQ가 HIGH 확인
    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_warn("vs1003_write_sci: DREQ not HIGH after writing to addr 0x%02x\n", addr);
        return ret;
    }

    return 0;
}

static int vs1003_read_sci(struct vs1003_data *dev, uint8_t addr, uint16_t *out_val) {
    int ret;
    uint8_t tx_buf[4] = {
        VS1003_READ_COMMAND,
        addr,
        0xFF,  // dummy
        0xFF   // dummy
    };
    uint8_t rx_buf[4] = {0};

    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("vs1003_read_sci: DREQ timeout before reading SCI\n");
        return ret;
    }

    xcs_select(dev);
    udelay(1);

    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,
        .speed_hz = VS1003_CMD_HZ,
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret = spi_sync(dev->spi, &m);
    
    udelay(1);
    xcs_deselect(dev);


    if (ret < 0) {
        pr_err("vs1003_read_sci: spi_sync failed\n");
        return ret;
    }

    if (out_val)
        *out_val = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];

    pr_debug("vs1003_read_sci: addr=0x%02x, val=0x%04x\n", addr, *out_val);
    return 0;
}

static void vs1003_xfer_work(struct work_struct *work) {

    struct vs1003_data *dev = container_of(work, struct vs1003_data, xfer_work);
    unsigned int copied;
    uint8_t tmp[VS1003_CHUNK_SIZE];
    int ret;


    while(true) {

        if (!READ_ONCE(dev->xfer_enabled))
            break;

        // 재생 중 볼륨 pending이 걸린 경우 먼저 처리
        // SDI를 멈추고 SCI를 먼저 처리하도록
        if (atomic_read(&dev->volume_pending)) {
            uint16_t target = READ_ONCE(dev->volume_target_byte);

            ret = vs1003_wait_dreq(dev, 200000);
            if (ret == 0) {

                // SCI 적용을 시도하는 순간에만 pending value 0으로
                if (atomic_xchg(&dev->volume_pending, 0)) {
                    mutex_lock(&dev->lock);
                    ret = vs1003_write_sci(dev, SCI_VOL, target);
                    mutex_unlock(&dev->lock);

                    if (ret == 0) {
                        dev->volume_byte = target;
                        pr_debug("xfer_work: volume applied 0x%04X\n", target);
                    } else {
                        // 실패시 pending 복구
                        atomic_set(&dev->volume_pending, 1);
                        pr_warn("xfer_work: volume apply failed ret=%d\n", ret);

                        // SDI 전송으로 돌아가지 않고 enqueue
                        queue_work(dev->wq, &dev->xfer_work);
                        break;
                    }
                }
            }
            else {
                // SCI 다시 시도
                queue_work(dev->wq, &dev->xfer_work);
                break;
            }
        }
        
        // 코덱이 SDI 받을 준비가 안 됐으면 다시 enqueue
        if (!gpiod_get_value(dev->dreq_gpio)) {
            queue_work(dev->wq, &dev->xfer_work);
            break;
        }

        // FIFO에서 buffer에서 data pop
        mutex_lock(&dev->fifo_lock);
        copied = kfifo_out(&dev->fifo, tmp, sizeof(tmp));
        mutex_unlock(&dev->fifo_lock);

        if (copied == 0) {
            // 더이상 보낼 데이터가 없음
            // writer 깨워줌
            // KFIFO에 공간이 생겼으므로 이것을 Noti함
            // 결과적으로 vs1003_write에서 kfifo의 공간이 부족해서 대기하는 프로세스가 접근함
            wake_up_interruptible(&dev->wq_space);

            if (atomic_read(&dev->play_state) != PLAY_STOPPED) {
                atomic_set(&dev->play_state, PLAY_STOPPED);
                pr_debug("xfer_work: state -> PLAY_STOPPED\n");
            }

            break;
        }


        // XDCS On 후 SPI 전송
        xdcs_select(dev);
        {
            struct spi_transfer t = {
                .tx_buf   = tmp,
                .len      = copied,
                .speed_hz = VS1003_STREAM_HZ, 
            };
            struct spi_message m;
            spi_message_init(&m);
            spi_message_add_tail(&t, &m);
            ret = spi_sync(dev->spi, &m);
        }
        xdcs_deselect(dev);

        if (ret < 0) {
            pr_err("xfer_work: spi_sync failed ret=%d\n", ret);
            break;
        }

        if (atomic_read(&dev->play_state) != PLAY_PLAYING) {
            atomic_set(&dev->play_state, PLAY_PLAYING);
        }

        // spi 전송 완료 후 fifo 공간이 생김에 따라 NOTI
        // 결과적으로 vs1003_write에서 kfifo의 공간이 부족해서 대기하는 프로세스가 접근함
        wake_up_interruptible(&dev->wq_space);

        // 전송 직후 volume pending이 있는 경우 continue 후 바로 volume set 시도하도록
        if (atomic_read(&dev->volume_pending)) {
            cond_resched();
            continue;
        }

        // DREQ가 계속 HIGH이고, FIFO 데이터가 남아있다면 잠시 양보함
        // 더 높은 우선순위 프로세스에게 양보
        cond_resched();
    }
}

static void vs1003_force_hard_reset(struct vs1003_data *dev) {

    pr_debug("vs1003_force_hard_reset\n");
    
    // 모든 제어 신호 deselsect로
    xcs_deselect(dev);
    xdcs_deselect(dev);
    
    // 하드웨어 리셋
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(10);
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100);
}

static int vs1003_soft_reset(struct vs1003_data *dev) {

    pr_debug("vs1003_soft_reset\n");
    int ret;
    uint16_t val;

    // reset + default mode 설정
    ret = vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW);
    if (ret < 0) {
        pr_err("vs1003_soft_reset: failed to write SCI_MODE\n");
        return ret;
    }

    msleep(2);

    // DREQ wait
    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("vs1003_soft_reset: DREQ timeout after reset\n");
        return ret;
    }

    // write Clock
    ret = vs1003_write_sci(dev, SCI_CLOCKF, VS1003_DEFAULT_CLOCKF);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_CLOCKF, &val);
    if (ret < 0 || val != VS1003_DEFAULT_CLOCKF) {
        pr_warn("SCI_CLOCKF mismatch: expected 0x%04x, got 0x%04x\n", VS1003_DEFAULT_CLOCKF, val);
    }

    // write audio data
    ret = vs1003_write_sci(dev, SCI_AUDATA, VS1003_DEFAULT_AUDATA);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_AUDATA, &val);
    if (ret < 0 || val != VS1003_DEFAULT_AUDATA) {
        pr_warn("SCI_AUDATA mismatch: expected 0x%04x, got 0x%04x\n", VS1003_DEFAULT_AUDATA, val);
    }

    uint16_t vol_reg = ((uint16_t)dev->volume_byte << 8) | dev->volume_byte; // 좌우 동일하게
    ret = vs1003_write_sci(dev, SCI_VOL, vol_reg);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_VOL, &val);
    if (ret < 0 || val != vol_reg) {
        pr_warn("SCI_VOL mismatch: expected 0x%04x, got 0x%04x\n", vol_reg, val);
    }

    dev->volume_byte = vol_reg;


    return 0;
}

static int vs1003_init_chip(struct vs1003_data *dev) {
    pr_info("vs1003_init_chip\n");
    
    vs1003_force_hard_reset(dev);

    int ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_err("vs1003_init_chip: DREQ timeout after hard reset\n");
        return ret;
    }

    uint8_t vol_byte = vs1003_vol_percent_to_byte(VS1003_DEFAULT_VOLUME_PER);
    uint16_t vol_reg = (vol_byte << 8) | vol_byte;
    dev->volume_byte = vol_reg;

    // volume pending 초기화
    atomic_set(&dev->volume_pending, 0);

    return vs1003_soft_reset(dev);
}

static int vs1003_init_kfifo(struct vs1003_data *dev) {
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

    // DREQ INT
    dev->dreq_irq = gpiod_to_irq(dev->dreq_gpio);
    if (dev->dreq_irq < 0) {
        pr_err("gpiod_to_irq(dreq) failed: %d\n", dev->dreq_irq);
        ret = dev->dreq_irq;
        goto err_wq;
    }

    ret = request_threaded_irq(dev->dreq_irq,
                               NULL,                     /* top half 없음 */
                               vs1003_dreq_irq_thread,   /* threaded handler */
                               IRQF_ONESHOT | IRQF_TRIGGER_RISING,
                               "vs1003-dreq", dev);
    if (ret) {
        pr_err("request_threaded_irq failed: %d\n", ret);
        goto err_wq;
    }

    dev->xfer_enabled = true;
    atomic_set(&dev->play_state, PLAY_STOPPED);

    // DREQ가 이미 HIGH인 경우
    // request_threaded_irq() 호출 직후 DREQ가 이미 HIGH라면
    // 하드웨어에 따라서는 엣지 트리거가 안 잡혀서 인터럽트가 바로 안 올 수 있음
    if (gpiod_get_value(dev->dreq_gpio)) {
        queue_work(dev->wq, &dev->xfer_work);
    }

    return 0;

err_wq:
   if (dev->wq) { 
    flush_workqueue(dev->wq); 
    destroy_workqueue(dev->wq); 
    dev->wq = NULL; 
}

err_fifo:
    if (dev->dreq_irq >= 0) {
         free_irq(dev->dreq_irq, dev); dev->dreq_irq = -EINVAL; 
    }
    kfifo_free(&dev->fifo);
    return ret;
}

/* 재생중인 MP3 Data를 안전하게 중단*/
static int vs1003_kfifo_stop(struct vs1003_data *dev) {
    int ret; 
    uint16_t mode;

    // read 현재 모드
    ret = vs1003_read_sci(dev, SCI_MODE, &mode); 
    if ( ret < 0) {
        return ret;
    }

    // write 현재 모드에서 CANCEL을 OR연산하여 파일 디코딩 중단
    ret = vs1003_write_sci(dev, SCI_MODE, mode | SM_CANCEL); 
    if ( ret < 0 )  {
        return ret;
    }

    // end-fill : 칩의 내부 버퍼를 비우기 위해 null byte 전송
    {
        uint8_t fill[32] = {0};
        int remain = 8192;
        while (remain > 0) {

            // DREQ wait
            ret = vs1003_wait_dreq(dev, 200000);
            if (ret < 0) {
                pr_err("vs1003_kfifo_stop: DREQ timeout\n");
                break;
            }

            xdcs_select(dev);
            {
                struct spi_transfer t = { 
                    .tx_buf = fill,
                    .len = sizeof(fill), 
                    .speed_hz = VS1003_STREAM_HZ 
                };

                struct spi_message m; 
                spi_message_init(&m); 
                spi_message_add_tail(&t, &m);
                spi_sync(dev->spi, &m);
                udelay(1);
            }
            xdcs_deselect(dev);
            remain -= sizeof(fill);
        }
    }

    // SM_CANCEL clear 확인 체크
    {
        int i; for (i=0;i<200;i++) {
            ret = vs1003_read_sci(dev, SCI_MODE, &mode); 
            if ( ret < 0) {
                break;
            } 

            if ((mode & SM_CANCEL) == 0) {
                break;
            }
            udelay(200);
        }
    }


    atomic_set(&dev->play_state, PLAY_STOPPED);

    return 0;
}

static void vs1003_kfifo_release(struct vs1003_data *dev) {

    dev->xfer_enabled = false;

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


static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {

    struct vs1003_data *dev = vs1003_dev;
    ssize_t written = 0;

    pr_debug("vs1003_write\n");

    if (count == 0)
        return 0;

    // 유저공간의 요청 데이터 (count)
    // 커널 버퍼에 복사함 
    while (written < count) {

        // remain byte 계산
        size_t to_copy = min_t(size_t, count - written, (size_t)VS1003_MAX_BUFFER_SIZE);
        unsigned int copied = 0;

        // fifo 공간이 생기거나, stream이 비활성화 될 때까지 대기
        if (wait_event_interruptible(dev->wq_space, kfifo_avail(&dev->fifo) > 0 || !READ_ONCE(dev->xfer_enabled))) {
            return -ERESTARTSYS;
        }

        if (!dev->xfer_enabled) {
            return -EPIPE;
        }

        // 유저버퍼를 직접 KFIFO 버퍼에 복사함
        mutex_lock(&dev->fifo_lock); 
        {
            int ret = kfifo_from_user(&dev->fifo, buf + written, to_copy, &copied);
            if (ret < 0) { mutex_unlock(&dev->fifo_lock); return ret; }
        }
        mutex_unlock(&dev->fifo_lock);

        written += copied;

        // worker 깨움
        queue_work(dev->wq, &dev->xfer_work);

        // 쓰기 못한 경우 
        if (copied == 0) {
            cond_resched();
        }
    }

    return written;
}

static int vs1003_open(struct inode *inode, struct file *file) {

    pr_debug("vs1003_open\n");

    if (!mutex_trylock(&vs1003_dev->lock)) {
        return -EBUSY;
    }

    if (vs1003_soft_reset(vs1003_dev) < 0) {
        pr_warn("vs1003_open: soft reset failed, trying hard reset\n");
        vs1003_force_hard_reset(vs1003_dev);

        if (vs1003_soft_reset(vs1003_dev) < 0) {
            pr_err("vs1003_open: hard reset also failed\n");
            mutex_unlock(&vs1003_dev->lock);
            return -EIO;
        }
    }
    return 0;
}

static int vs1003_release(struct inode *inode, struct file *file) {
    pr_debug("vs1003_release\n");

    mutex_unlock(&vs1003_dev->lock);
    return 0;
}

static const struct file_operations vs1003_fops = {
    .owner = THIS_MODULE,
    .open = vs1003_open,
    .write = vs1003_write,
    .release = vs1003_release,
};

static int vs1003_probe(struct spi_device *spi) {

    pr_debug("vs1003_probe\n");
    int ret;
    struct device *dev = &spi->dev;

    vs1003_dev = devm_kzalloc(dev, sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev)
        return -ENOMEM;

    spi->mode = SPI_MODE_0;
    spi->mode |= SPI_NO_CS; // CS 비활성
    spi->max_speed_hz = VS1003_STREAM_HZ;
    spi->bits_per_word = 8;

    ret = spi_setup(spi);
    if (ret) {
        pr_err("vs1003_probe: spi_setup failed\n");
        return ret;
    }

     vs1003_dev->spi = spi;
    mutex_init(&vs1003_dev->lock);

    /* GPIO 설정 */
    vs1003_dev->dreq_gpio = devm_gpiod_get(dev, "dreq", GPIOD_IN);
    if (IS_ERR(vs1003_dev->dreq_gpio)) {
        pr_err("vs1003_probe: failed to get dreq gpio\n");
        return PTR_ERR(vs1003_dev->dreq_gpio);
    }

    vs1003_dev->xcs_gpio = devm_gpiod_get(dev, "xcs", GPIOD_OUT_LOW);
    if (IS_ERR(vs1003_dev->xcs_gpio)) {
        pr_err("vs1003_probe: failed to get xcs gpio\n");
        return PTR_ERR(vs1003_dev->xcs_gpio);
    }

    vs1003_dev->xdcs_gpio = devm_gpiod_get(dev, "xdcs", GPIOD_OUT_LOW);
    if (IS_ERR(vs1003_dev->xdcs_gpio)) {
        pr_err("vs1003_probe: failed to get xdcs gpio\n");
        return PTR_ERR(vs1003_dev->xdcs_gpio);
    }

    vs1003_dev->xrst_gpio = devm_gpiod_get(dev, "xrst", GPIOD_OUT_HIGH);
    if (IS_ERR(vs1003_dev->xrst_gpio)) {
        pr_err("vs1003_probe: failed to get xrst gpio\n");
        return PTR_ERR(vs1003_dev->xrst_gpio);
    }

    /* 문자 디바이스 등록 */
    ret = alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("vs1003_probe: alloc_chrdev_region failed\n");
        return ret;
    }

    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    ret = cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);
    if (ret < 0) {
        pr_err("vs1003_probe: cdev_add failed\n");
        goto err_chrdev;
    }

    vs1003_dev->class = class_create(CLASS_NAME);
    if (IS_ERR(vs1003_dev->class)) {
        pr_err("vs1003_probe: class_create failed\n");
        ret = PTR_ERR(vs1003_dev->class);
        goto err_cdev;
    }

    vs1003_dev->device = device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);
    if (IS_ERR(vs1003_dev->device)) {
        pr_err("device_create failed\n");
        ret = PTR_ERR(vs1003_dev->device);
        goto err_class;
    }

    /* sysfs: drvdata 연결 + 그룹 생성 */
    dev_set_drvdata(vs1003_dev->device, vs1003_dev);
    ret = sysfs_create_group(&vs1003_dev->device->kobj, &vs1003_group);
    if (ret) {
        pr_err("sysfs_create_group failed: %d\n", ret);
        goto err_device;
    }

    // vs1003 칩 초기화
    ret = vs1003_init_chip(vs1003_dev);
    if (ret < 0) {
        pr_err("vs1003_probe: chip initialization failed\n");
        goto err_sysfs;
    }

    // kfifo 초기화
    ret = vs1003_init_kfifo(vs1003_dev);
    if (ret < 0) {
        pr_err("vs1003_probe: kfifo initialization failed\n");
        goto err_sysfs;
    }


    pr_debug("vs1003_probe: successfully initialized\n");
    return 0;

err_sysfs:
    sysfs_remove_group(&vs1003_dev->device->kobj, &vs1003_group);
err_device:
    device_destroy(vs1003_dev->class, vs1003_dev->devt);
err_class:
    class_destroy(vs1003_dev->class);
err_cdev:
    cdev_del(&vs1003_dev->cdev);
err_chrdev:
    unregister_chrdev_region(vs1003_dev->devt, 1);
    return ret;
}

static void vs1003_remove(struct spi_device *spi) {

    pr_debug("vs1003_remove\n");

    vs1003_kfifo_stop(vs1003_dev);

    vs1003_kfifo_release(vs1003_dev);

    if (vs1003_dev->device)
        sysfs_remove_group(&vs1003_dev->device->kobj, &vs1003_group);

    device_destroy(vs1003_dev->class, vs1003_dev->devt);
    class_destroy(vs1003_dev->class);
    cdev_del(&vs1003_dev->cdev);
    unregister_chrdev_region(vs1003_dev->devt, 1);

}

static const struct of_device_id vs1003_of_match[] = {
    { .compatible = "vs1003" }, {},
};

MODULE_DEVICE_TABLE(of, vs1003_of_match);

static struct spi_driver vs1003_driver = {
    .driver = {
        .name = DEV_NAME,
        .of_match_table = vs1003_of_match,
    },
    .probe = vs1003_probe,
    .remove = vs1003_remove,
};

static int __init vs1003_init(void) {

    pr_debug("vs1003_init\n");
    return spi_register_driver(&vs1003_driver);
}

static void __exit vs1003_exit(void) {

    pr_debug("vs1003_exit\n");
    spi_unregister_driver(&vs1003_driver);
}

module_init(vs1003_init);
module_exit(vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chan");
MODULE_DESCRIPTION("VS1003 Linux SPI Character Driver");