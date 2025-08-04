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



cat test.mp3 | sudo tee /dev/vs1003 > /dev/null
head -c 1024  test.mp3 | sudo tee /dev/vs1003 > /dev/null
*/

#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"

#define VS1003_BUF_SIZE 1024 

#define MAX_ALLOWED_BYTES (128 * 1024)  // 예시
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

// VS1003 초기화
static int vs1003_init_chip(struct vs1003_data *dev)
{
    int timeout = 1000;

    // 하드웨어 리셋
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(20);  // 더 긴 리셋 시간
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100); // 더 긴 대기 시간


    // DREQ 대기
    timeout = 1000;
    while (timeout-- > 0 && !gpiod_get_value(dev->dreq_gpio)) {
        msleep(1);
    }
    if (timeout <= 0) {
        return -ENODEV; 
    }

    // 소프트 리셋
    vs1003_write_sci(dev, SCI_MODE, 0x0804);  // SM_SDINEW + SM_RESET
    msleep(100);
    
    // DREQ 대기
    timeout = 1000;
    while (timeout-- > 0 && !gpiod_get_value(dev->dreq_gpio)) {
        msleep(1);
    }
    if (timeout <= 0) {
        return -ENODEV; 
    }

    // 클럭 설정 (더 안정적인 값)
    vs1003_write_sci(dev, SCI_CLOCKF, 0x6000);  // 기본 클럭
    msleep(10);
    
    // 볼륨 설정 (더 높은 볼륨)
    vs1003_write_sci(dev, SCI_VOL, 0x1010);     // -16dB
    
    return 0;
}

#define VS1003_CHUNK_SIZE 128  // MP3 프레임 단위 맞춤
static ssize_t vs1003_write_stream(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct vs1003_data *dev = file->private_data;
    u8 tmp[VS1003_CHUNK_SIZE];
    size_t written = 0;

    if (count > MAX_ALLOWED_BYTES) {
        count = MAX_ALLOWED_BYTES;
    }
    
    pr_debug("vs1003_write_stream: starting to write %zu bytes\n", count);

    while (written < count) {
        int timeout = 10000; // 10초로 증가 (10000 * 1ms)
        
        // DREQ 대기
        while (!gpiod_get_value(dev->dreq_gpio)) {
            if (timeout-- <= 0) {
                pr_err("vs1003_write_stream: DREQ timeout after %zu bytes\n", written);
                return written > 0 ? written : -ETIMEDOUT;
            }
            usleep_range(100, 200); // 100-200μs 대기 (더 정밀한 대기)
        }
        
        size_t to_copy = min(count - written, (size_t)VS1003_CHUNK_SIZE);
        if (copy_from_user(tmp, buf + written, to_copy)) {
            pr_err("vs1003_write_stream: copy_from_user failed at %zu bytes\n", written);
            return written > 0 ? written : -EFAULT;
        }
        
        // SPI 전송
        gpiod_set_value(dev->dcs_gpio, 0);
        
        struct spi_transfer xfer = {
            .tx_buf = tmp,
            .len = to_copy,
        };
        struct spi_message msg;
        spi_message_init(&msg);
        spi_message_add_tail(&xfer, &msg);
        
        
        int ret = spi_sync(dev->spi, &msg);
        if (ret < 0) {
            pr_err("vs1003_write_stream: spi_sync failed: %d\n", ret);
            return written > 0 ? written : ret;
        }

        written += to_copy;

        if ((written % 1024) == 0 || written == count) {
            pr_debug("vs1003_write_stream: written %zu/%zu bytes\n", written, count);
        }
    }
    
    pr_debug("vs1003_write_stream: completed %zu bytes\n", written);
    return written;
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    pr_debug("vs1003_open: device opened\n");
    file->private_data = vs1003_dev;
    return 0;
}

static int vs1003_release(struct inode *inode, struct file *file)
{
    struct vs1003_data *dev = file->private_data;
    pr_info("vs1003_release: device closed\n");

    // 스트리밍 종료를 위한 soft reset 또는 SM_CANCEL 명령 전송
    vs1003_write_sci(dev, SCI_MODE, 0x080C); // SM_SDINEW | SM_RESET
        
    // 최대 100ms 동안 대기
    for (int i = 0; i < 100; i++) {
        if (gpiod_get_value(dev->dreq_gpio))
            break;
        msleep(1);
    }

    return 0;
}

static const struct file_operations vs1003_fops = {
    .owner = THIS_MODULE,
    .open = vs1003_open,
    .write = vs1003_write_stream,
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
    spi->max_speed_hz = 1000000;  // 1MHz로 시작
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