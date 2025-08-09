#define pr_fmt(fmt) "[VS1003] " fmt

//#define DEBUG

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

#define VS1003_FIFO_SIZE   (64 * 1024)   // 64KB 권장
#define VS1003_CHUNK_SIZE  32            

#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"


#define VS1003_MAX_BUFFER_SIZE 4096
#define VS1003_WRITE_COMMAND 0x02
#define VS1003_READ_COMMAND 0x03
#define DEFAULT_VOLUME 0x1010

#define SCI_MODE    0x00
#define SCI_CLOCKF  0x03
#define SCI_AUDATA  0x05
#define SCI_VOL     0x0B

#define SM_RESET    0x04
#define SM_SDINEW   0x0800

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


    // KFIFO Streaming
    struct workqueue_struct *wq;
    struct kfifo fifo;
    struct mutex fifo_lock;          // FIFO 보호
    wait_queue_head_t wq_space;    // FIFO 공간 대기
    struct work_struct xfer_work;  // 전송 워커
    int dreq_irq;                  // DREQ IRQ 번호
    bool streaming_enabled;        // 전송 가능 상태 플래그
};

static struct vs1003_data *vs1003_dev;

static int vs1003_soft_reset(struct vs1003_data *dev);
static void vs1003_force_hard_reset(struct vs1003_data *dev);


// xcs와 xdcs는 dts에서 active low이므로 코드상 설정과 물리적 출력이 반대임
static inline void xcs_select(struct vs1003_data *d)   { gpiod_set_value(d->xcs_gpio, 1); }
static inline void xcs_deselect(struct vs1003_data *d) { gpiod_set_value(d->xcs_gpio, 0); }

static inline void xdcs_select(struct vs1003_data *d)   { gpiod_set_value(d->xdcs_gpio, 1); }
static inline void xdcs_deselect(struct vs1003_data *d) { gpiod_set_value(d->xdcs_gpio, 0); }


static int vs1003_wait_dreq(struct vs1003_data *dev, int usec_timeout) {
    int dreq_val;
    while (usec_timeout-- > 0) {
        dreq_val = gpiod_get_value(dev->dreq_gpio);
        if (dreq_val)
            return 0;
        udelay(1);
    }
    // 타임아웃 시 DREQ 상태를 로그에 남김
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

static int vs1003_write_sci(struct vs1003_data *dev, u8 addr, u16 data)
{
    int ret;
    
    pr_debug("vs1003_write_sci: addr=0x%02x, data=0x%04x\n", addr, data);

    u8 tx_buf[4] = {
        VS1003_WRITE_COMMAND,  // 0x02
        addr,                  // 레지스터 주소
        (data >> 8) & 0xFF,    // 상위 바이트
        data & 0xFF            // 하위 바이트
    };

    // 전송 전 DREQ가 HIGH 상태인지 확인
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
        .speed_hz = 500000, // 속도를 줄여서 안정성 향상
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    // SPI 전송 수행
    ret = spi_sync(dev->spi, &m);
    
    // 전송 완료 후 지연
    udelay(1);
    
    // XCS HIGH (쓰기 종료)
    xcs_deselect(dev);

    if (ret < 0) {
        pr_err("vs1003_write_sci: spi_sync failed for addr 0x%02x, ret=%d\n", addr, ret);
        return ret;
    }

    // 전송 후 DREQ가 다시 HIGH 되는지 확인
    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) {
        pr_warn("vs1003_write_sci: DREQ not HIGH after writing to addr 0x%02x\n", addr);
        return ret;
    }

    return 0;
}

static int vs1003_read_sci(struct vs1003_data *dev, u8 addr, u16 *out_val)
{
    int ret;
    u8 tx_buf[4] = {
        VS1003_READ_COMMAND,
        addr,
        0xFF,  // dummy
        0xFF   // dummy
    };
    u8 rx_buf[4] = {0};

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
        .speed_hz = 500000, // 읽기 속도도 감소
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
        *out_val = ((u16)rx_buf[2] << 8) | rx_buf[3];

    pr_debug("vs1003_read_sci: addr=0x%02x, val=0x%04x\n", addr, *out_val);
    return 0;
}

static void vs1003_xfer_work(struct work_struct *work) 
{
    struct vs1003_data *dev = container_of(work, struct vs1003_data, xfer_work);
    unsigned int copied;

    if (!dev->streaming_enabled)
        return;

    u8 tmp[VS1003_CHUNK_SIZE];
    int ret;

    while(true) {

        // DREQ가 LOW면 멈춤
        if (!gpiod_get_value(dev->dreq_gpio)) {
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
            break;
        }

        // XDCS On 후 SPI 전송
        xdcs_select(dev);
        {
            struct spi_transfer t = {
                .tx_buf   = tmp,
                .len      = copied,
                .speed_hz = 3000000, // 2Mhz
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

        // spi 전송 완료 후 fifo 공간이 생김에 따라 NOTI
        // 결과적으로 vs1003_write에서 kfifo의 공간이 부족해서 대기하는 프로세스가 접근함
        wake_up_interruptible(&dev->wq_space);

        // DREQ가 계속 HIGH이고, FIFO 데이터가 남아있다면 잠시 양보함
        // 더 높은 우선순위 프로세스에게 양보
        cond_resched();
    }
}


/* SDI로 임의 바이트 블록을 전송하는 유틸리티 */
static int vs1003_sdi_write(struct vs1003_data *dev, const u8 *buf, size_t len)
{
    int ret;

    /* DREQ가 HIGH일 때만 SDI 전송 */
    ret = vs1003_wait_dreq(dev, 100000);
    if (ret < 0) return ret;

    xdcs_select(dev);
    {
        struct spi_transfer t = {
            .tx_buf   = buf,
            .len      = len,
            .speed_hz = 3000000, /* SDI 3MHz */
        };
        struct spi_message m;
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        ret = spi_sync(dev->spi, &m);
    }
    xdcs_deselect(dev);

    return ret;
}

/* 사인파 재생 시작
 * code: 사인파 코드(예: 0x44 ≈ 1kHz 근처)
 * vol : 볼륨(SCI_VOL 값, 0x0000=최대, 0xFEFE=무음 방향)
 */
static int vs1003_start_sine(struct vs1003_data *dev, u8 code, u16 vol)
{
    int ret;
    u16 mode;

    /* 스트리밍 경로는 잠시 비활성화(충돌 방지) */
    WRITE_ONCE(dev->streaming_enabled, false);
    flush_work(&dev->xfer_work);

    /* TEST 모드 진입: SM_TESTS set */
    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;

    ret = vs1003_write_sci(dev, SCI_MODE, mode | 0x0020 /* SM_TESTS */);
    if (ret < 0) return ret;

    /* 필요시 볼륨 설정 (기본 0x1010 → 적당히 조정 가능) */
    if (vol)
        vs1003_write_sci(dev, SCI_VOL, vol);

    /* 사인파 시작 시퀀스 (8바이트): 0x53 0xEF 0x6E code 0 0 0 0 */
    {
        u8 sine_on[8] = { 0x53, 0xEF, 0x6E, code, 0x00, 0x00, 0x00, 0x00 };
        ret = vs1003_sdi_write(dev, sine_on, sizeof(sine_on));
        if (ret < 0) return ret;
    }

    return 0;
}

/* 사인파 재생 종료 */
static int vs1003_stop_sine(struct vs1003_data *dev)
{
    int ret;
    u16 mode;

    /* 종료 시퀀스 (8바이트): 0x45 0x78 0x69 0x74 0 0 0 0  ("Exit") */
    {
        u8 sine_off[8] = { 0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00 };
        ret = vs1003_sdi_write(dev, sine_off, sizeof(sine_off));
        if (ret < 0) return ret;
    }

    /* TEST 모드 해제: SM_TESTS clear */
    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;

    ret = vs1003_write_sci(dev, SCI_MODE, mode & ~0x0020 /* ~SM_TESTS */);
    if (ret < 0) return ret;

    /* 스트리밍 재개를 원하면 여기서 true로 */
    WRITE_ONCE(dev->streaming_enabled, true);

    return 0;
}


static void vs1003_force_hard_reset(struct vs1003_data *dev)
{
    pr_debug("vs1003_force_hard_reset\n");
    
    // 모든 제어 신호를 안전한 상태로
    xcs_deselect(dev);
    xdcs_deselect(dev);
    
    // 하드웨어 리셋
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(10);
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100);
}

static int vs1003_soft_reset(struct vs1003_data *dev)
{
    pr_debug("vs1003_soft_reset\n");
    int ret;
    u16 val;

    ret = vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW);
    if (ret < 0) {
        pr_err("vs1003_soft_reset: failed to write SCI_MODE\n");
        return ret;
    }
    
    msleep(2);

    int timeout = 1000;
    while (timeout-- && !gpiod_get_value(dev->dreq_gpio))
        udelay(10);
    if (timeout <= 0) {
        pr_err("vs1003_soft_reset: DREQ timeout after reset\n");
        return -EIO;
    }

    // 클록 설정
    ret = vs1003_write_sci(dev, SCI_CLOCKF, 0x8800);
    if (ret < 0) return ret;
    
    msleep(1); // 설정 후 잠시 대기
    
    ret = vs1003_read_sci(dev, SCI_CLOCKF, &val);
    if (ret < 0 || val != 0x8800) {
        pr_warn("SCI_CLOCKF mismatch: expected 0x8800, got 0x%04x\n", val);
    }

    // 오디오 데이터 설정
    ret = vs1003_write_sci(dev, SCI_AUDATA, 0xAC45);
    if (ret < 0) return ret;
    
    msleep(1);
    
    ret = vs1003_read_sci(dev, SCI_AUDATA, &val);
    if (ret < 0 || val != 0xAC45) {
        pr_warn("SCI_AUDATA mismatch: expected 0xAC45, got 0x%04x\n", val);
    }

    // 볼륨 설정
    ret = vs1003_write_sci(dev, SCI_VOL, DEFAULT_VOLUME);
    if (ret < 0) return ret;
    
    msleep(1);
    
    ret = vs1003_read_sci(dev, SCI_VOL, &val);
    if (ret < 0 || val != DEFAULT_VOLUME) {
        pr_warn("SCI_VOL mismatch: expected 0x%04x, got 0x%04x\n", DEFAULT_VOLUME, val);
    }

    return 0;
}

static int vs1003_init_chip(struct vs1003_data *dev)
{
    pr_info("vs1003_init_chip\n");
    
    vs1003_force_hard_reset(dev);
    
    if (!gpiod_get_value(dev->dreq_gpio)) {
        pr_err("vs1003_init_chip: DREQ not HIGH after reset\n");
        return -ENODEV;
    }

    return vs1003_soft_reset(dev);
}

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

    dev->streaming_enabled = true;

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

static int vs1003_kfifo_stop(struct vs1003_data *dev)
{
    int ret; u16 mode;

    /* SM_CANCEL set */
    ret = vs1003_read_sci(dev, SCI_MODE, &mode); if (ret<0) return ret;
    ret = vs1003_write_sci(dev, SCI_MODE, mode | 0x0008 /* SM_CANCEL */); if (ret<0) return ret;

    /* end-fill: 0x00 다량 주입 (DREQ를 기다리며) */
    {
        u8 fill[32] = {0};
        int remain = 8192; /* 8KB 전송 목표 */
        while (remain > 0) {
            /* DREQ가 LOW면 잠깐 대기 */
            int to = 2000; /* ~2ms 대기 루프 */
            while (!gpiod_get_value(dev->dreq_gpio) && --to)
                udelay(1);
            if (!gpiod_get_value(dev->dreq_gpio)) break; /* 더 못 보냄 */

            xdcs_select(dev);
            {
                struct spi_transfer t = { .tx_buf = fill, .len = sizeof(fill), .speed_hz = 3000000 };
                struct spi_message m; spi_message_init(&m); spi_message_add_tail(&t, &m);
                spi_sync(dev->spi, &m);
            }
            xdcs_deselect(dev);
            remain -= sizeof(fill);
        }
    }

    /* SM_CANCEL clear 확인 (최대 대기) */
    {
        int i; for (i=0;i<200;i++) { /* ~40ms */
            ret = vs1003_read_sci(dev, SCI_MODE, &mode); if (ret<0) break;
            if ((mode & 0x0008) == 0) break;
            udelay(200);
        }
    }

    /* (옵션) 소프트 리셋/볼륨 mute 한 번 살짝 */
    // vs1003_write_sci(dev, SCI_VOL, 0xFEFE);

    return 0;
}

static void vs1003_kfifo_release(struct vs1003_data *dev)
{

    // vs1003_stop_sine(vs1003_dev);


    dev->streaming_enabled = false;


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
        if (wait_event_interruptible(dev->wq_space, kfifo_avail(&dev->fifo) > 0 || !READ_ONCE(dev->streaming_enabled))) {
            return -ERESTARTSYS;
        }

        if (!dev->streaming_enabled) {
            return -EPIPE;
        }

        // user buffer에 복사
        // {
        //     u8 tmp[VS1003_MAX_BUFFER_SIZE];
        //     if (copy_from_user(tmp, buf + written, to_copy)) {
        //         return -EFAULT;
        //     }

        //     // kfifo 버퍼에 데이터 push
        //     spin_lock(&dev->fifo_lock);
        //     copied = kfifo_in(&dev->fifo, tmp, to_copy);
        //     spin_unlock(&dev->fifo_lock);
        // }

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

static int vs1003_open(struct inode *inode, struct file *file)
{
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

static int vs1003_release(struct inode *inode, struct file *file)
{
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

static int vs1003_probe(struct spi_device *spi)
{
    pr_debug("vs1003_probe\n");
    int ret;
    struct device *dev = &spi->dev;

    vs1003_dev = devm_kzalloc(dev, sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev)
        return -ENOMEM;

    spi->mode = SPI_MODE_0;
    spi->mode |= SPI_NO_CS; // CS 비활성
    spi->max_speed_hz = 2000000; // 최대 속도 증가
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret) {
        pr_err("vs1003_probe: spi_setup failed\n");
        return ret;
    }

    vs1003_dev->spi = spi;
    mutex_init(&vs1003_dev->lock);

    // GPIO 설정 개선 - 더 구체적인 에러 처리
    vs1003_dev->dreq_gpio = devm_gpiod_get(dev, "dreq", GPIOD_IN);
    if (IS_ERR(vs1003_dev->dreq_gpio)) {
        pr_err("vs1003_probe: failed to get dreq gpio\n");
        return PTR_ERR(vs1003_dev->dreq_gpio);
    }

    vs1003_dev->xcs_gpio = devm_gpiod_get(dev, "xcs", GPIOD_OUT_LOW); // dts active low이므로 low=물리적 high
    if (IS_ERR(vs1003_dev->xcs_gpio)) {
        pr_err("vs1003_probe: failed to get xcs gpio\n");
        return PTR_ERR(vs1003_dev->xcs_gpio);
    }

    vs1003_dev->xdcs_gpio = devm_gpiod_get(dev, "xdcs", GPIOD_OUT_LOW); // dts active low이므로 low=물리적 high
    if (IS_ERR(vs1003_dev->xdcs_gpio)) {
        pr_err("vs1003_probe: failed to get xdcs gpio\n");
        return PTR_ERR(vs1003_dev->xdcs_gpio);
    }

    vs1003_dev->xrst_gpio = devm_gpiod_get(dev, "xrst", GPIOD_OUT_HIGH);
    if (IS_ERR(vs1003_dev->xrst_gpio)) {
        pr_err("vs1003_probe: failed to get xrst gpio\n");
        return PTR_ERR(vs1003_dev->xrst_gpio);
    }

    // 문자 디바이스 등록
    ret = alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("vs1003_probe: alloc_chrdev_region failed\n");
        return ret;
    }

    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    ret = cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);
    if (ret < 0) {
        pr_err("vs1003_probe: cdev_add failed\n");
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return ret;
    }

    vs1003_dev->class = class_create(CLASS_NAME);
    if (IS_ERR(vs1003_dev->class)) {
        pr_err("vs1003_probe: class_create failed\n");
        cdev_del(&vs1003_dev->cdev);
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return PTR_ERR(vs1003_dev->class);
    }

    device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);
    
    // 칩 초기화
    ret = vs1003_init_chip(vs1003_dev);
    if (ret < 0) {
        pr_err("vs1003_probe: chip initialization failed\n");
        device_destroy(vs1003_dev->class, vs1003_dev->devt);
        class_destroy(vs1003_dev->class);
        cdev_del(&vs1003_dev->cdev);
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return ret;
    }

    // kfifo 초기화
    ret = vs1003_init_kfifo(vs1003_dev);
    if(ret < 0) {
        pr_err("vs1003_probe: kfifo initialization failed\n");
        device_destroy(vs1003_dev->class, vs1003_dev->devt);
        class_destroy(vs1003_dev->class);
        cdev_del(&vs1003_dev->cdev);
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return ret;
    }


    //vs1003_start_sine(vs1003_dev, 0x44, 0x2020);

    pr_debug("vs1003_probe: successfully initialized\n");
    return 0;
}

static void vs1003_remove(struct spi_device *spi)
{
    pr_debug("vs1003_remove\n");

    vs1003_kfifo_stop(vs1003_dev);

    vs1003_kfifo_release(vs1003_dev);

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

static int __init vs1003_init(void)
{
    pr_debug("vs1003_init\n");
    return spi_register_driver(&vs1003_driver);
}

static void __exit vs1003_exit(void)
{
    pr_debug("vs1003_exit\n");
    spi_unregister_driver(&vs1003_driver);
}

module_init(vs1003_init);
module_exit(vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chan");
MODULE_DESCRIPTION("VS1003 Linux SPI Driver with Improved Timing");