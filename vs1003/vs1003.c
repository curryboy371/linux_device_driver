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


/*



cat music.mp3 | sudo tee /dev/vs1003 > /dev/null

*/

#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"

#define VS1003_BUF_SIZE 1024 


#define SCI_MODE    0x00
#define SCI_STATUS 0x01
#define SCI_CLOCKF  0x03
#define SCI_VOL     0x0B

struct vs1003_data {
    struct spi_device *spi;
    struct gpio_desc *dcs_gpio;  // XDCS
    struct gpio_desc *dreq_gpio;
    struct gpio_desc *xrst_gpio;
    struct class *class;
    struct cdev cdev;
    dev_t devt;
    struct mutex lock;
};

static struct vs1003_data *vs1003_dev;

// SCI 읽기 함수 추가
static u16 vs1003_read_sci(struct vs1003_data *dev, u8 addr)
{

    // DREQ 대기
    int timeout = 1000;
    while (timeout-- > 0 && !gpiod_get_value(dev->dreq_gpio)) {
        udelay(10);
    }
    
    if (timeout <= 0) {
        pr_warn("vs1003_read_sci: DREQ timeout\n");
    }
    
    u8 tx_buf[4] = { 0x03, addr, 0x00, 0x00 }; // READ + ADDR + 2 dummy bytes
    u8 rx_buf[4] = { 0, 0, 0, 0 };
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,  // 4바이트로 수정
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(dev->spi, &msg);
    
    // 마지막 2바이트에서 실제 데이터 추출
    u16 val = (rx_buf[2] << 8) | rx_buf[3];
    
    pr_info("vs1003_read_sci: addr=0x%02x, received val=0x%04x\n", addr, val);
    
    return val;
}

static void vs1003_write_sci(struct vs1003_data *dev, u8 addr, u16 val)
{
    pr_debug("vs1003_write_sci: addr=0x%02x, val=0x%04x\n", addr, val); // Log added

    u8 buf[4] = { 0x02, addr, val >> 8, val & 0xFF };
    struct spi_transfer xfer = {
        .tx_buf = buf,
        .len = 4,
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(dev->spi, &msg);  // SPI0_CE0 자동 사용됨

    udelay(2);
}

// 1. 상세한 SPI 읽기 디버깅
static u16 vs1003_read_sci_debug(struct vs1003_data *dev, u8 addr)
{
    // DREQ 상태 확인
    int dreq_val = gpiod_get_value(dev->dreq_gpio);
    pr_info("vs1003_read_sci_debug: DREQ=%d before transaction\n", dreq_val);
    
    // DREQ 대기
    int timeout = 1000;
    while (timeout-- > 0 && !gpiod_get_value(dev->dreq_gpio)) {
        udelay(10);
    }
    
    if (timeout <= 0) {
        pr_warn("vs1003_read_sci_debug: DREQ timeout\n");
        return 0xFFFF;
    }
    
    u8 tx_buf[4] = { 0x03, addr, 0x00, 0x00 };
    u8 rx_buf[4] = { 0xFF, 0xFF, 0xFF, 0xFF };  // 초기값을 0xFF로 설정
    
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,
        .speed_hz = 1000000,  // 1MHz로 속도 제한
        .bits_per_word = 8,
        .cs_change = 0,
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    
    pr_info("vs1003_read_sci_debug: Before SPI - TX=[%02x %02x %02x %02x], RX=[%02x %02x %02x %02x]\n", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    
    int ret = spi_sync(dev->spi, &msg);
    
    pr_info("vs1003_read_sci_debug: After SPI (ret=%d) - TX=[%02x %02x %02x %02x], RX=[%02x %02x %02x %02x]\n", 
            ret, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    
    // DREQ 상태 재확인
    dreq_val = gpiod_get_value(dev->dreq_gpio);
    pr_info("vs1003_read_sci_debug: DREQ=%d after transaction\n", dreq_val);
    
    u16 val = (rx_buf[2] << 8) | rx_buf[3];
    pr_info("vs1003_read_sci_debug: addr=0x%02x, final val=0x%04x\n", addr, val);
    
    return val;
}

// 2. SPI 설정 확인 함수
static void vs1003_check_spi_config(struct vs1003_data *dev)
{
    struct spi_device *spi = dev->spi;
    
    pr_info("vs1003_spi_config: max_speed_hz=%d\n", spi->max_speed_hz);
    pr_info("vs1003_spi_config: mode=0x%02x\n", spi->mode);
    pr_info("vs1003_spi_config: bits_per_word=%d\n", spi->bits_per_word);
    pr_info("vs1003_spi_config: chip_select=%d\n", spi->chip_select);
    
    // GPIO 상태 확인
    pr_info("vs1003_gpio_status: DREQ=%d, XDCS=%d, XRST=%d\n",
            gpiod_get_value(dev->dreq_gpio),
            gpiod_get_value(dev->dcs_gpio),
            gpiod_get_value(dev->xrst_gpio));
}

// 3. 루프백 테스트 함수 (VS1003 없이 SPI 동작 확인)
static int vs1003_spi_loopback_test(struct vs1003_data *dev)
{
    u8 tx_buf[4] = { 0xAA, 0x55, 0xCC, 0x33 };
    u8 rx_buf[4] = { 0x00, 0x00, 0x00, 0x00 };
    
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,
        .speed_hz = 1000000,
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    
    pr_info("vs1003_loopback: Before - TX=[%02x %02x %02x %02x]\n", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    
    int ret = spi_sync(dev->spi, &msg);
    
    pr_info("vs1003_loopback: After (ret=%d) - RX=[%02x %02x %02x %02x]\n", 
            ret, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    
    return ret;
}

// 4. 개선된 초기화 함수
static int vs1003_init_chip_debug(struct vs1003_data *dev)
{
    int timeout = 1000;
    u16 status, mode;
    
    pr_info("vs1003_init_debug: Starting initialization\n");
    
    // SPI 설정 확인
    vs1003_check_spi_config(dev);
    
    // SPI 루프백 테스트
    vs1003_spi_loopback_test(dev);
    
    // 1. 하드웨어 리셋
    pr_info("vs1003_init_debug: Hardware reset\n");
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(20);  // 더 긴 리셋 시간
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100); // 부팅 시간 충분히 대기
    
    // 2. DREQ 대기
    while (timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio)) {
            pr_info("vs1003_init_debug: DREQ HIGH after %d ms\n", 1000-timeout);
            break;
        }
        msleep(1);
    }
    
    if (timeout <= 0) {
        pr_err("vs1003_init_debug: DREQ timeout\n");
        return -ENODEV;
    }
    
    // 3. 초기 상태 확인 (디버그 모드)
    pr_info("vs1003_init_debug: Reading initial registers\n");
    status = vs1003_read_sci_debug(dev, SCI_STATUS);
    mode = vs1003_read_sci_debug(dev, SCI_MODE);
    
    // 4. 간단한 쓰기 테스트 먼저
    pr_info("vs1003_init_debug: Testing simple write\n");
    vs1003_write_sci(dev, SCI_VOL, 0x4040);  // 간단한 볼륨 설정
    msleep(10);
    
    u16 vol_readback = vs1003_read_sci_debug(dev, SCI_VOL);
    pr_info("vs1003_init_debug: Volume write test - wrote 0x4040, read 0x%04x\n", vol_readback);
    
    if (vol_readback == 0x4040) {
        pr_info("vs1003_init_debug: SPI communication working!\n");
        return 0;
    } else {
        pr_err("vs1003_init_debug: SPI communication failed\n");
        return -EIO;
    }
}

// VS1003 초기화
static int vs1003_init_chip(struct vs1003_data *dev)
{
    int timeout = 1000;
    u16 status, mode;
    
    // 1. 하드웨어 리셋
    gpiod_set_value(dev->xrst_gpio, 0);  // LOW
    msleep(10);
    gpiod_set_value(dev->xrst_gpio, 1);  // HIGH
    msleep(10);
    
    pr_debug("vs1003: XRST completed\n");
    
    // 2. DREQ가 HIGH가 될 때까지 대기
    while (timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio)) {
            pr_debug("vs1003: DREQ is HIGH, ready for communication\n");
            break;
        }
        msleep(1);
    }
    
    if (timeout <= 0) {
        pr_err("vs1003: DREQ timeout - chip may not be responding\n");
        return -ENODEV;
    }
    
    // 3. 초기화 전 상태 확인
    status = vs1003_read_sci(dev, SCI_STATUS);
    pr_debug("vs1003: Initial STATUS: 0x%04x\n", status);
    
    mode = vs1003_read_sci(dev, SCI_MODE);
    pr_debug("vs1003: Initial MODE: 0x%04x\n", mode);
    
    // 4. 소프트웨어 리셋
    vs1003_write_sci(dev, SCI_MODE, 0x0804);  // SM_SDINEW + SM_RESET
    msleep(10);
    
    // 5. 리셋 후 DREQ 대기
    timeout = 1000;
    while (timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio)) {
            break;
        }
        msleep(1);
    }
    
    if (timeout <= 0) {
        pr_err("vs1003: DREQ timeout after soft reset\n");
        return -ENODEV;
    }
    
    // 6. 클럭 설정
    vs1003_write_sci(dev, SCI_CLOCKF, 0x9800);  // 3x clock
    msleep(10);
    
    // 7. 볼륨 설정
    vs1003_write_sci(dev, SCI_VOL, 0x2020);     // -32dB
    
    // 8. 최종 상태 확인
    status = vs1003_read_sci(dev, SCI_STATUS);
    mode = vs1003_read_sci(dev, SCI_MODE);
    
    pr_debug("vs1003: Final STATUS: 0x%04x, MODE: 0x%04x\n", status, mode);
    
    // STATUS가 여전히 0이면 문제가 있음
    if (status == 0x0000) {
        pr_err("vs1003: STATUS register still 0x0000 - check connections\n");
        return -ENODEV;
    }
    
    return 0;
}

static ssize_t vs1003_write_stream(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct vs1003_data *dev = file->private_data;
    u8 tmp[VS1003_BUF_SIZE];
    size_t written = 0;

    pr_debug("vs1003_write_stream: starting to write %zu bytes\n", count); // Log added

    while (written < count) {
        // dreq가 high일 때만 전송 가능
        if (!gpiod_get_value(dev->dreq_gpio)) {
            pr_debug("vs1003_write_stream: DREQ is LOW, waiting...\n"); // Log added
            udelay(10);
            continue;
        }


        // 데이터 복사
        size_t to_copy = min(count - written, (size_t)VS1003_BUF_SIZE);
        if (copy_from_user(tmp, buf + written, to_copy))
            return -EFAULT;

        // spi 전송
        struct spi_transfer xfer = {
            .tx_buf = tmp,
            .len = to_copy,
        };
        struct spi_message msg;


        // XDCS LOW (active-low이므로 0으로 설정하면 활성화)
        gpiod_set_value(dev->dcs_gpio, 0);

        spi_message_init(&msg);
        spi_message_add_tail(&xfer, &msg);

         // SPI 동기 전송
        spi_sync(dev->spi, &msg);

        // XDCS HIGH (비활성화)
        gpiod_set_value(dev->dcs_gpio, 1);

        written += to_copy;
    }

    return written;
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    pr_debug("vs1003_open: device opened\n");
    file->private_data = vs1003_dev;
    return 0;
}

static const struct file_operations vs1003_fops = {
    .owner = THIS_MODULE,
    .open = vs1003_open,
    .write = vs1003_write_stream,
};

static int vs1003_probe(struct spi_device *spi)
{
    int ret;
    struct device *dev = &spi->dev;

    pr_debug("vs1003: probe started\n");

    vs1003_dev = devm_kzalloc(dev, sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev) {
        pr_err("vs1003: Failed to allocate memory\n");
        return -ENOMEM;
    }

    vs1003_dev->spi = spi;
    mutex_init(&vs1003_dev->lock);

    vs1003_dev->dcs_gpio = devm_gpiod_get(dev, "dcs", GPIOD_OUT_HIGH);
    if (IS_ERR(vs1003_dev->dcs_gpio)) {
        pr_err("vs1003: Failed to get dcs-gpio\n");
        return PTR_ERR(vs1003_dev->dcs_gpio);
    }

    vs1003_dev->dreq_gpio = devm_gpiod_get(dev, "dreq", GPIOD_IN);
    if (IS_ERR(vs1003_dev->dreq_gpio)) {
        pr_err("vs1003: Failed to get dreq-gpio\n");
        return PTR_ERR(vs1003_dev->dreq_gpio);
    }

    // XRST GPIO 제어 추가
    vs1003_dev->xrst_gpio = devm_gpiod_get(dev, "xrst", GPIOD_OUT_LOW);
    if (IS_ERR(vs1003_dev->xrst_gpio)) {
        pr_err("vs1003: Failed to get xrst-gpio\n");
        return PTR_ERR(vs1003_dev->dreq_gpio);
    }


    ret = alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("vs1003: Failed to alloc chrdev region\n");
        return ret;
    }

    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    ret = cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);
    if (ret < 0) {
        pr_err("vs1003: Failed to add cdev\n");
        goto unregister_region;
    }

    vs1003_dev->class = class_create(CLASS_NAME);
    if (IS_ERR(vs1003_dev->class)) {
        pr_err("vs1003: Failed to create class\n");
        ret = PTR_ERR(vs1003_dev->class);
        goto del_cdev;
    }


    // GPIO 핀 번호 확인
    pr_info("vs1003: GPIO pins - DCS:%d, DREQ:%d, XRST:%d\n",
            desc_to_gpio(vs1003_dev->dcs_gpio),
            desc_to_gpio(vs1003_dev->dreq_gpio),
            desc_to_gpio(vs1003_dev->xrst_gpio));

    // GPIO 초기값 확인
    pr_info("vs1003: GPIO values - DCS:%d, DREQ:%d, XRST:%d\n",
            gpiod_get_value(vs1003_dev->dcs_gpio),
            gpiod_get_value(vs1003_dev->dreq_gpio),
            gpiod_get_value(vs1003_dev->xrst_gpio));

    device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);

    if(vs1003_init_chip_debug(vs1003_dev)) {
        pr_err("vs1003: Failed to vs1003_init_chip\n");
        goto del_cdev;
    }

    if(vs1003_init_chip(vs1003_dev)) {
        pr_err("vs1003: Failed to vs1003_init_chip\n");
        goto del_cdev;
    }


    pr_info("vs1003: driver loaded successfully\n");
    return 0;

del_cdev:
    cdev_del(&vs1003_dev->cdev);
unregister_region:
    unregister_chrdev_region(vs1003_dev->devt, 1);
    return ret;
}

static void vs1003_remove(struct spi_device *spi)
{
    pr_info("vs1003: driver removed\n");

    if (!vs1003_dev) {
        pr_warn("vs1003: vs1003_dev is NULL in remove\n");
        return;
    }

    // 1. 디바이스 파일 제거 (사용자 접근 차단)
    if (vs1003_dev->class && vs1003_dev->devt) {
        device_destroy(vs1003_dev->class, vs1003_dev->devt);
        pr_info("vs1003: device destroyed\n");
    }

    // 2. 클래스 제거
    if (vs1003_dev->class) {
        class_destroy(vs1003_dev->class);
        vs1003_dev->class = NULL;
        pr_info("vs1003: class destroyed\n");
    }

    // 3. 문자 디바이스 제거
    cdev_del(&vs1003_dev->cdev);
    pr_info("vs1003: cdev deleted\n");

    // 4. 디바이스 번호 해제
    if (vs1003_dev->devt) {
        unregister_chrdev_region(vs1003_dev->devt, 1);
        pr_info("vs1003: chrdev region unregistered\n");
    }

    // 5. 전역 포인터 정리
    vs1003_dev = NULL;

    pr_info("vs1003: driver removed successfully\n");
}


static const struct of_device_id vs1003_of_match[] = {
    { .compatible = "vs1003" },
    { }
};
MODULE_DEVICE_TABLE(of, vs1003_of_match);

// static const struct spi_device_id vs1003_id[] = {
//     { "vs1003", 0 },
//     { }
// };
// MODULE_DEVICE_TABLE(spi, vs1003_id);

static struct spi_driver vs1003_driver = {
    .driver = {
        .name = DEV_NAME,
        .of_match_table = vs1003_of_match,
    },
    .probe = vs1003_probe,
    .remove = vs1003_remove,
    //.id_table = vs1003_id,
};

module_spi_driver(vs1003_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ChatGPT");
MODULE_DESCRIPTION("VS1003 MP3 Codec SPI Driver");