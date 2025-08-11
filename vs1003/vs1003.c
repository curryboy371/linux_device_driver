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
    struct mutex ctl_lock;

    struct mutex spi_lock;
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

static int vs1003_diag_selftest(struct vs1003_data *dev);

// static inline uint8_t vs1003_vol_percent_to_byte(unsigned int percent);
// static inline unsigned int vs1003_vol_byte_to_percent(uint8_t v);

static ssize_t volume_percent_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t volume_percent_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static int vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data, bool bwait);
static int vs1003_write_sdi(struct vs1003_data *dev, const uint8_t *buf, size_t len);


static bool vs1003_sci_work(struct vs1003_data* dev);
static bool vs1003_sdi_work(struct vs1003_data *dev, size_t budget);


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

    // sci 설정은 항상 work queue에서 동작하도록
    WRITE_ONCE(vs1003_dev->volume_target_byte, vol_reg);
    atomic_set(&vs1003_dev->volume_pending, 1);
    queue_work(vs1003_dev->wq, &vs1003_dev->xfer_work);
    pr_debug("volume_percent_store: pending ...\n");
    return count;
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

        // fifo에 data가 남은 경우 (재생중)
        // volume pending이 1인 경우 ( volume 설정 예약됨 )
        // 재생중인 경우
        if (READ_ONCE(dev->xfer_enabled) && (!kfifo_is_empty(&dev->fifo) || atomic_read(&dev->volume_pending) || atomic_read(&dev->play_state) == PLAY_PLAYING)) {
            queue_work(dev->wq, &dev->xfer_work);
        }
    }

    return IRQ_HANDLED;
}

static int vs1003_write_sci(struct vs1003_data *dev, uint8_t addr, uint16_t data, bool bwait) {
    int ret;
    
    pr_debug("vs1003_write_sci: addr=0x%02x, data=0x%04x\n", addr, data);

    uint8_t tx_buf[4] = {
        VS1003_WRITE_COMMAND,  // 0x02
        addr,                  // 레지스터 주소
        (data >> 8) & 0xFF,    // 상위 바이트
        data & 0xFF            // 하위 바이트
    };

    // 전송 전 DREQ가 HIGH 확인

    if(bwait) {
        ret = vs1003_wait_dreq(dev, 100000);
        if (ret < 0) {
            pr_err("vs1003_write_sci: DREQ timeout before writing to addr 0x%02x\n", addr);
            return ret;
        }
    }
    else {
        // work가 재시도
        if (!gpiod_get_value(dev->dreq_gpio))
            return -EBUSY;
    }


    xdcs_deselect(dev);
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

    if(bwait) {
        // 전송 후 DREQ가 HIGH 확인
        ret = vs1003_wait_dreq(dev, 100000);
        if (ret < 0) {
            pr_warn("vs1003_write_sci: DREQ not HIGH after writing to addr 0x%02x\n", addr);
            return ret;
        }
    }
    else {
        // work가 재시도
        if (!gpiod_get_value(dev->dreq_gpio))
            return -EBUSY;
    }

    return 0;
}
static int vs1003_write_sdi(struct vs1003_data *dev, const uint8_t *buf, size_t len) {

    int ret = 0;

    if (!len) return 0;

    /* DREQ 준비 대기 (너무 오래 묶이지 않게 타임아웃 짧게) */
    ret = vs1003_wait_dreq(dev, 200000);
    if (ret < 0) return ret;

    mutex_lock(&dev->spi_lock);
    xcs_deselect(dev);   /* SDI 전용: SCI CS는 반드시 HIGH */
    xdcs_select(dev);
    {
        struct spi_transfer t = {
            .tx_buf   = buf,
            .len      = len,
            .speed_hz = VS1003_STREAM_HZ,
        };
        struct spi_message m;
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        ret = spi_sync(dev->spi, &m);
    }
    xdcs_deselect(dev);
    mutex_unlock(&dev->spi_lock);
    return ret;
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

    bool did_any = false;


    while(true) {

        if (!READ_ONCE(dev->xfer_enabled))
            break;

        did_any = false;

        // SCI 명령 우선 처리
        if (vs1003_sci_work(dev)) {
            did_any = true;
        }

        // SDI 명령 처리

        if (!kfifo_is_empty(&dev->fifo)) {

            for (int i = 0; i < 4; ++i) {
                
                if (!vs1003_sdi_work(dev, 32)) {
                    break;
                }
                
                did_any = true;

                // spi 전송 완료 후 fifo 공간이 생김에 따라 NOTI
                // 결과적으로 vs1003_xfer_work에서  kfifo의 공간이 부족해서 대기하는 스레드가 접근
                wake_up_interruptible(&dev->wq_space);

                if (atomic_read(&dev->play_state) != PLAY_PLAYING) {
                    atomic_set(&dev->play_state, PLAY_PLAYING);
                }

                if (atomic_read(&dev->volume_pending)) {
                    break; 
                }
            }
        }

        if (!did_any) {
            
            // fifo queue가 비어있는지 확인 
            // queue가 비었으면서 stop이 아니었다면 stop으로 전환
            if (kfifo_is_empty(&dev->fifo) && atomic_read(&dev->play_state) != PLAY_STOPPED) {
                atomic_set(&dev->play_state, PLAY_STOPPED);
                pr_debug("xfer_work: state -> PLAY_STOPPED\n");
            }
            break;
        }

        if (atomic_read(&dev->volume_pending)) {
            cond_resched();
            continue;
        }

        // 양보
        cond_resched();
    }
}

// work queue 실행 후 sci를 write하는 함수
static bool vs1003_sci_work(struct vs1003_data* dev) {

    // atomic하게 pending 0으로 변환, 0이 아닌 경우 종료
    if (!atomic_xchg(&dev->volume_pending, 0)) {
        return false;
    }

    // dreq가 준비 안된 경우 다시 pending 복구
    if (!gpiod_get_value(dev->dreq_gpio)) {
        atomic_set(&dev->volume_pending, 1);
        return false;
    }

    // 최신 target value 얻어옴
    const uint16_t target = READ_ONCE(dev->volume_target_byte);

    mutex_lock(&dev->spi_lock);
    int ret = vs1003_write_sci(dev, SCI_VOL, target, false);
    mutex_unlock(&dev->spi_lock);

    if(ret == 0) {
        WRITE_ONCE(dev->volume_byte, target);
        pr_debug("vs1003_sci_write: SCI_VOL 0x%04x\n", READ_ONCE(target));
        return true;
    }

    // write 실패시 pending 다시 복구
    atomic_set(&dev->volume_pending, 1);
    pr_warn("sci_work: SCI_VOL apply failed ret=%d\n", ret);
    return false;
}

// work queue 실행 후 sdi를 write하는 함수
static bool vs1003_sdi_work(struct vs1003_data *dev, size_t budget) {

    uint8_t  chunk[32];
    int ret = 0;

    if (!gpiod_get_value(dev->dreq_gpio))
        return false;

    if (kfifo_is_empty(&dev->fifo))
        return false;

    unsigned int copied_len = kfifo_out(&dev->fifo, chunk, min_t(size_t, sizeof(chunk), budget));

    // 더이상 보낼 데이터가 없음
    if (copied_len == 0) {
        return false;
    }

    mutex_lock(&dev->spi_lock);

    xcs_deselect(dev);
    xdcs_select(dev);
    {
        struct spi_transfer t = {
            .tx_buf   = chunk,
            .len      = copied_len,
            .speed_hz = VS1003_STREAM_HZ,
        };
        struct spi_message m;
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        ret = spi_sync(dev->spi, &m);
    }
    xdcs_deselect(dev);
    mutex_unlock(&dev->spi_lock);

    if (ret < 0) {
        pr_err("sdi_work: spi_sync failed ret=%d\n", ret);
        return false;
    }

    return true;
}

// test sign tone start
static int vs1003_diag_sine_start(struct vs1003_data *dev, uint8_t freq_code)
{
    int ret;
    uint16_t mode;

    /* SM_TESTS 세트 */
    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;
    ret = vs1003_write_sci(dev, SCI_MODE, mode | SM_TESTS, true);
    if (ret < 0) return ret;

    /* 시퀀스: 0x53 0xEF 0x6E <freq> 0 0 0 0 */
    {
        uint8_t seq[8] = {0x53, 0xEF, 0x6E, freq_code, 0x00, 0x00, 0x00, 0x00};
        ret = vs1003_write_sdi(dev, seq, sizeof(seq));
        if (ret < 0) return ret;
    }

    pr_info("DIAG SINE: start, code=0x%02x\n", freq_code);
    return 0;
}

// test sign tone stop
static int vs1003_diag_sine_stop(struct vs1003_data *dev)
{
    uint16_t ret;
    u16 mode;

    /* 종료 시퀀스: 0x45 0x78 0x69 0x74 0 0 0 0 */
    {
        uint8_t seq[8] = {0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00};
        ret = vs1003_write_sdi(dev, seq, sizeof(seq));
        if (ret < 0) return ret;
    }

    /* SM_TESTS 클리어 */
    ret = vs1003_read_sci(dev, SCI_MODE, &mode);
    if (ret < 0) return ret;
    ret = vs1003_write_sci(dev, SCI_MODE, mode & ~SM_TESTS, true);
    if (ret < 0) return ret;

    pr_info("DIAG SINE: stop\n");
    return 0;
}

static int vs1003_diag_selftest(struct vs1003_data *dev)
{
    int ret = 0;

    ret = vs1003_diag_sine_start(dev, 0x24); /* 1kHz */
    if (ret < 0) {
        return ret;
    }

    // 2 sec
    msleep(2000);

    vs1003_diag_sine_stop(dev);

    return ret;
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
    ret = vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW, true);
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
    ret = vs1003_write_sci(dev, SCI_CLOCKF, VS1003_DEFAULT_CLOCKF, true);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_CLOCKF, &val);
    if (ret < 0 || val != VS1003_DEFAULT_CLOCKF) {
        pr_warn("SCI_CLOCKF mismatch: expected 0x%04x, got 0x%04x\n", VS1003_DEFAULT_CLOCKF, val);
    }

    // write audio data
    ret = vs1003_write_sci(dev, SCI_AUDATA, VS1003_DEFAULT_AUDATA, true);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_AUDATA, &val);
    if (ret < 0 || val != VS1003_DEFAULT_AUDATA) {
        pr_warn("SCI_AUDATA mismatch: expected 0x%04x, got 0x%04x\n", VS1003_DEFAULT_AUDATA, val);
    }

    ret = vs1003_write_sci(dev, SCI_VOL, dev->volume_byte, true);
    if (ret < 0) {
        return ret;
    }
    msleep(1);
    ret = vs1003_read_sci(dev, SCI_VOL, &val);
    if (ret < 0 || val != dev->volume_byte) {
        pr_warn("SCI_VOL mismatch: expected 0x%04x, got 0x%04x\n", dev->volume_byte, val);
    }

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

    uint8_t vol_single_byte = vs1003_vol_percent_to_byte(VS1003_DEFAULT_VOLUME_PER);
    uint16_t vol_reg = (vol_single_byte << 8) | vol_single_byte;
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
    ret = vs1003_write_sci(dev, SCI_MODE, mode | SM_CANCEL, true); 
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


static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {

    struct vs1003_data *dev = vs1003_dev;
    ssize_t written = 0;

    // pr_debug("vs1003_write\n");

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

    if (!mutex_trylock(&vs1003_dev->ctl_lock)) {
        return -EBUSY;
    }

    if (vs1003_soft_reset(vs1003_dev) < 0) {
        pr_warn("vs1003_open: soft reset failed, trying hard reset\n");
        vs1003_force_hard_reset(vs1003_dev);

        if (vs1003_soft_reset(vs1003_dev) < 0) {
            pr_err("vs1003_open: hard reset also failed\n");
            mutex_unlock(&vs1003_dev->ctl_lock);
            return -EIO;
        }
    }
    return 0;
}

static int vs1003_release(struct inode *inode, struct file *file) {
    pr_debug("vs1003_release\n");

    mutex_unlock(&vs1003_dev->ctl_lock);
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
    mutex_init(&vs1003_dev->ctl_lock);
    mutex_init(&vs1003_dev->spi_lock);

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


    vs1003_diag_selftest(vs1003_dev);

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

    flush_work(&vs1003_dev->xfer_work);

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