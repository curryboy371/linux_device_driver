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

#include "vs1003_def.h"

/*



cat test.mp3 | sudo tee /dev/vs1003 > /dev/null
head -c 1024  test.mp3 | sudo tee /dev/vs1003 > /dev/null
*/

#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"

#define VS1003_BUF_SIZE 32

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

static const char *vs1003_register_name[] = {
    "SCI_MODE",
    "SCI_STATUS",
    "SCI_BASS",
    "SCI_CLOCKF",
    "SCI_DECTIME",
    "SCI_AUDATA",
    "SCI_WRAM",
    "SCI_WRAMADDR",
    "SCI_HDAT0",
    "SCI_HDAT1",
    "SCI_AIADDR",
    "SCI_VOL",
    "SCI_AICTRL0",
    "SCI_AICTRL1",
    "SCI_AICTRL2",
    "SCI_AICTRL3"
};

#define NUM_REGISTERS (sizeof(vs1003_register_name) / sizeof(vs1003_register_name[0]))


static struct vs1003_data *vs1003_dev;

static int vs1003_soft_reset(struct vs1003_data *dev);

int vs1003_wait_for_dreq(struct vs1003_data *dev, int timeout_us)
{
    int waited = 0;

    while (!gpiod_get_value(dev->dreq_gpio)) {
        if (waited >= timeout_us) {
            pr_err("vs1003_wait_for_dreq: Timeout\n");
            return -ETIMEDOUT;
        }
        usleep_range(100, 200);  // 더 여유 있게 대기
        waited += 100;
    }
    return 0;
}

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
    
    u8 tx_buf[4] = { VS1003_READ_COMMAND, addr, 0x00, 0x00 }; // READ + ADDR + 2 dummy bytes
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


static void vs1003_dump_registers(struct vs1003_data *dev)
{
    int i;
    u16 val;
    pr_info("=== VS1003 SCI Register Dump ===\n");

    for (i = 0; i < NUM_REGISTERS; i++) {
        val = vs1003_read_sci(dev, i);
        pr_info("  %-12s : 0x%04X\n", vs1003_register_name[i], val);
    }
}


static void vs1003_write_sci(struct vs1003_data *dev, u8 addr, u16 val)
{
    pr_debug("vs1003_write_sci: addr=0x%02x, val=0x%04x\n", addr, val);

    u8 buf[VS1003_CMD_LEN] = { VS1003_WRITE_COMMAND, addr, val >> 8, val & 0xFF };
    struct spi_transfer xfer = {
        .tx_buf = buf,
        .len = VS1003_CMD_LEN,
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(dev->spi, &msg);
    udelay(2);
}


// VS1003 초기화
static int vs1003_init_chip(struct vs1003_data *dev)
{
    int timeout;

    pr_info("vs1003_init_chip: resetting chip\n");
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(10);
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100);

    timeout = 1000;
    while (timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio))
            break;
        msleep(1);
    }
    if (timeout <= 0) {
        pr_err("vs1003_init_chip: DREQ did not go HIGH after reset\n");
        return -ENODEV;
    }

    pr_info("vs1003_init_chip: sending SM_RESET\n");
    vs1003_write_sci(dev, SCI_MODE, SM_SDINEW | SM_RESET);
    msleep(100);

    timeout = 1000;
    while (timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio))
            break;
        msleep(1);
    }
    if (timeout <= 0) {
        pr_err("vs1003_init_chip: DREQ did not recover after soft reset\n");
        return -ENODEV;
    }

    pr_info("vs1003_init_chip: configuring CLOCKF\n");
    vs1003_write_sci(dev, SCI_CLOCKF, 0x6000);
    msleep(10);

    pr_info("vs1003_init_chip: setting volume\n");
    vs1003_write_sci(dev, SCI_VOL, 0xFEFE);

    pr_info("vs1003_init_chip: initialization complete\n");
    return 0;
}

static ssize_t vs1003_write_stream(struct vs1003_data *dev, const char __user *buf, size_t count)
{
    u8 data[VS1003_BUF_SIZE];
    size_t remaining = count;
    int ret = 0;

    pr_info("vs1003_write_stream: starting to write %zu bytes\n", count);

    while (remaining > 0) {
        int to_copy = min((size_t)VS1003_BUF_SIZE, remaining);

        if (vs1003_wait_for_dreq(dev, 200000) < 0) {  // 50ms 대기
            pr_err("DREQ timeout after %zu bytes\n", count - remaining);
            ret = -EIO;
            break;
        }

        if (copy_from_user(data, buf + (count - remaining), to_copy)) {
            pr_err("vs1003_write_stream: copy_from_user failed\n");
            ret = -EFAULT;
            break;
        }

        gpiod_set_value(dev->dcs_gpio, 0);
        spi_write(dev->spi, data, to_copy);
        gpiod_set_value(dev->dcs_gpio, 1);

        remaining -= to_copy;
        pr_debug("vs1003_write_stream: written %zu/%zu bytes\n", count - remaining, count);
    }

    return ret ? ret : count - remaining;
}

static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    if (!vs1003_dev)
        return -ENODEV;

    return vs1003_write_stream(vs1003_dev, buf, count);
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    pr_info("vs1003_open: device opened\n");
    file->private_data = vs1003_dev;

    // DREQ 상태 확인 (칩이 데이터 수신 가능 상태인지)
    int timeout = 100;
    while (timeout-- > 0) {
        if (gpiod_get_value(vs1003_dev->dreq_gpio)) {
            pr_info("vs1003_open: DREQ is HIGH, ready to stream\n");
            return 0;
        }
        usleep_range(100, 200);  // 100~200us 대기
    }

    pr_warn("vs1003_open: DREQ not HIGH after wait, attempting soft reset\n");

    // soft reset 시도
    if (vs1003_soft_reset(vs1003_dev) < 0) {
        pr_err("vs1003_open: DREQ still LOW after reset, device busy\n");
        return -EBUSY;
    }
    pr_info("vs1003_open: device ready after soft reset\n");
    return 0;
}

static int vs1003_soft_reset(struct vs1003_data *dev)
{
    pr_info("vs1003_soft_reset: Performing soft reset\n");
    vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW);
    msleep(2);

    int timeout = 1000;
    while (timeout-- > 0 && !gpiod_get_value(dev->dreq_gpio))
        udelay(10);

    if (timeout <= 0) {
        pr_err("vs1003_soft_reset: DREQ not HIGH after soft reset\n");
        return -EIO;
    }

    // 기본 레지스터 재설정
    vs1003_write_sci(dev, SCI_CLOCKF, 0x6000);
    vs1003_write_sci(dev, SCI_AUDATA, 0x1F40);  // 8kHz stereo
    vs1003_write_sci(dev, SCI_VOL, 0x2020);     // 기본 볼륨

    // 확인용
    if (!gpiod_get_value(dev->dreq_gpio)) {
        pr_err("vs1003_soft_reset: DREQ not HIGH after soft reset");
    } else {
        pr_info("vs1003_soft_reset: DREQ is HIGH after reset");
    }

    return 0;
}

static int vs1003_release(struct inode *inode, struct file *file) {

    pr_info("vs1003_release: device closed\n");

    // SM_CANCEL 보내서 현재 스트리밍 취소
    pr_info("vs1003_release: sending SM_CANCEL\n");
    vs1003_write_sci(vs1003_dev, SCI_MODE, SM_CANCEL | SM_SDINEW);
    msleep(1);

    int timeout = 1000;
    while (timeout-- > 0) {
        if (gpiod_get_value(vs1003_dev->dreq_gpio))
            break;
        udelay(10);
    }

    if (timeout <= 0) {
        pr_warn("vs1003_release: DREQ not HIGH after SM_CANCEL, performing soft reset\n");
        vs1003_soft_reset(vs1003_dev);
    }

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
    int ret;
    struct device *dev = &spi->dev;

    pr_debug("vs1003: probe started\n");

    vs1003_dev = devm_kzalloc(dev, sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev) {
        pr_err("vs1003: Failed to allocate memory\n");
        return -ENOMEM;
    }

    
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 500000;  // 1MHz로 시작
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret < 0) {
        pr_err("vs1003: SPI setup failed\n");
        return ret;
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

    // GPIO 초기값 확인
    pr_info("vs1003: GPIO values - DCS:%d, DREQ:%d, XRST:%d\n",
            gpiod_get_value(vs1003_dev->dcs_gpio),
            gpiod_get_value(vs1003_dev->dreq_gpio),
            gpiod_get_value(vs1003_dev->xrst_gpio));

    device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);

    if(vs1003_init_chip(vs1003_dev)) {
        pr_err("vs1003: Failed to vs1003_init_chip\n");
        goto del_cdev;
    }

    // GPIO 초기값 확인
    pr_info("vs1003: GPIO values - DCS:%d, DREQ:%d, XRST:%d\n",
            gpiod_get_value(vs1003_dev->dcs_gpio),
            gpiod_get_value(vs1003_dev->dreq_gpio),
            gpiod_get_value(vs1003_dev->xrst_gpio));


    vs1003_dump_registers(vs1003_dev);

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