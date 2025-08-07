#define pr_fmt(fmt) "[VS1003] " fmt

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

#define DEV_NAME "vs1003"
#define CLASS_NAME "vs1003_class"
#define VS1003_CHUNK_SIZE 64
#define VS1003_MAX_BUFFER_SIZE 4096
#define VS1003_WRITE_COMMAND 0x02
#define VS1003_READ_COMMAND 0x03
#define DEFAULT_VOLUME 0x2020

#define SCI_MODE    0x00
#define SCI_CLOCKF  0x03
#define SCI_AUDATA  0x05
#define SCI_VOL     0x0B

#define SM_RESET    0x04
#define SM_SDINEW   0x0800

struct vs1003_data {
    struct spi_device *spi;
    struct gpio_desc* dreq_gpio;
    struct gpio_desc* xdcs_gpio;
    struct gpio_desc* xrst_gpio;
    struct class *class;
    struct cdev cdev;
    dev_t devt;
    struct mutex lock;
    u8 *stream_buf;
    size_t stream_len, stream_pos;
};

static struct vs1003_data *vs1003_dev;

static int vs1003_wait_dreq(struct vs1003_data *dev, int usec_timeout)
{
    while (usec_timeout-- > 0) {
        if (gpiod_get_value(dev->dreq_gpio))
            return 0;
        udelay(1);
    }
    return -EIO;
}

static int vs1003_write_sci(struct vs1003_data *dev, u8 addr, u16 data)
{
    pr_info("vs1003_write_sci: addr=0x%02x, data=0x%04x\n", addr, data);
    u8 tx_buf[4] = {
        VS1003_WRITE_COMMAND,
        addr,
        (data >> 8) & 0xff,
        data & 0xff,
    };
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = 4,
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spi_sync(dev->spi, &m);
}

static int vs1003_write_stream_chunk(struct vs1003_data *dev, const u8 *buf, size_t count)
{
    //pr_info("vs1003_write_stream_chunk: count=%zu\n", count);

    // 전송 전 DREQ 대기
    if (vs1003_wait_dreq(dev, 100000) < 0) {
        pr_err("vs1003_write_stream_chunk: DREQ timeout before XDCS LOW\n");
        return -EIO;
    }

    gpiod_set_value(dev->xdcs_gpio, 0);

    struct spi_transfer t = {
        .tx_buf = buf,
        .len = count,
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    int ret = spi_sync(dev->spi, &m);

    gpiod_set_value(dev->xdcs_gpio, 1);

    // 전송 후 DREQ가 다시 HIGH 될 때까지 대기
    if (vs1003_wait_dreq(dev, 100000) < 0) {
        pr_err("vs1003_write_stream_chunk: DREQ timeout after XDCS HIGH\n");
        return -EIO;
    }

    return ret;
}

static void vs1003_force_hard_reset(struct vs1003_data *dev)
{
    pr_info("vs1003_force_hard_reset\n");
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(10); // 최소 2ms 이상
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100); // 초기화 대기
}

static int vs1003_soft_reset(struct vs1003_data *dev)
{
    pr_info("vs1003_soft_reset\n");
    vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW);
    msleep(2);

    int timeout = 1000;
    while (timeout-- && !gpiod_get_value(dev->dreq_gpio))
        udelay(10);
    if (timeout <= 0)
        return -EIO;

    vs1003_write_sci(dev, SCI_CLOCKF, 0x6000);
    vs1003_write_sci(dev, SCI_AUDATA, 0x1F40);
    vs1003_write_sci(dev, SCI_VOL, DEFAULT_VOLUME);
    return 0;
}

static int vs1003_init_chip(struct vs1003_data *dev)
{
    pr_info("vs1003_init_chip\n");
    gpiod_set_value(dev->xrst_gpio, 0);
    msleep(10);
    gpiod_set_value(dev->xrst_gpio, 1);
    msleep(100);

    if (!gpiod_get_value(dev->dreq_gpio))
        return -ENODEV;

    return vs1003_soft_reset(dev);
}

static ssize_t vs1003_write_stream(struct vs1003_data *dev, const char __user *buf, size_t count)
{
    static int call_count = 0;
    pr_info("vs1003_write_stream: called %d, count = %zu\n", ++call_count, count);

    if (count > VS1003_MAX_BUFFER_SIZE)
        return -EINVAL;

    if (!dev->stream_buf) {
        dev->stream_buf = kmalloc(VS1003_MAX_BUFFER_SIZE, GFP_KERNEL);
        if (!dev->stream_buf)
            return -ENOMEM;
    }

    if (copy_from_user(dev->stream_buf, buf, count))
        return -EFAULT;

    dev->stream_len = count;
    dev->stream_pos = 0;

    while (dev->stream_pos < dev->stream_len) {
        size_t remain = dev->stream_len - dev->stream_pos;
        size_t to_write = min(remain, (size_t)VS1003_CHUNK_SIZE);

        if (remain > (size_t)VS1003_CHUNK_SIZE) {
            pr_debug("vs1003_write_stream: remain=%zu, sending chunk of %zu\n", remain, to_write);
        }

        if (vs1003_write_stream_chunk(dev, dev->stream_buf + dev->stream_pos, to_write) < 0) {
            pr_err("vs1003_write_stream: stream_chunk failed, resetting chip\n");
            vs1003_force_hard_reset(dev);
            vs1003_soft_reset(dev);
            break;
        }

        dev->stream_pos += to_write;
    }

    return dev->stream_pos;
}

static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    pr_info("vs1003_write\n");
    return vs1003_write_stream(vs1003_dev, buf, count);
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    pr_info("vs1003_open\n");


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
    pr_info("vs1003_release\n");
    if (vs1003_dev->stream_buf) {
        kfree(vs1003_dev->stream_buf);
        vs1003_dev->stream_buf = NULL;
    }
    vs1003_soft_reset(vs1003_dev);
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
    pr_info("vs1003_probe\n");
    int ret;
    struct device *dev = &spi->dev;

    vs1003_dev = devm_kzalloc(dev, sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev)
        return -ENOMEM;

    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 1000000;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret)
        return ret;

    vs1003_dev->spi = spi;
    mutex_init(&vs1003_dev->lock);

    vs1003_dev->xdcs_gpio = devm_gpiod_get(dev, "xdcs", GPIOD_OUT_HIGH);
    vs1003_dev->dreq_gpio = devm_gpiod_get(dev, "dreq", GPIOD_IN);
    vs1003_dev->xrst_gpio = devm_gpiod_get(dev, "xrst", GPIOD_OUT_LOW);

    if (IS_ERR(vs1003_dev->xdcs_gpio) || IS_ERR(vs1003_dev->dreq_gpio) || IS_ERR(vs1003_dev->xrst_gpio))
        return -EINVAL;

    ret = alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    if (ret < 0)
        return ret;

    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    ret = cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);
    if (ret < 0) {
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return ret;
    }

    vs1003_dev->class = class_create(CLASS_NAME);
    if (IS_ERR(vs1003_dev->class)) {
        cdev_del(&vs1003_dev->cdev);
        unregister_chrdev_region(vs1003_dev->devt, 1);
        return PTR_ERR(vs1003_dev->class);
    }

    device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);
    return vs1003_init_chip(vs1003_dev);
}

static void vs1003_remove(struct spi_device *spi)
{
    pr_info("vs1003_remove\n");
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
    pr_info("vs1003_init\n");
    return spi_register_driver(&vs1003_driver);
}

static void __exit vs1003_exit(void)
{
    pr_info("vs1003_exit\n");
    spi_unregister_driver(&vs1003_driver);
}

module_init(vs1003_init);
module_exit(vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chan");
MODULE_DESCRIPTION("VS1003 Linux SPI Driver with Polling");
