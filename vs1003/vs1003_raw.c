#define pr_fmt(fmt) "[VS1003_RAW] " fmt

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
#include <linux/platform_device.h>
#include "my_spi.h"

#define DEV_NAME "vs1003_raw"
#define CLASS_NAME "vs1003_class_raw"
#define VS1003_CHUNK_SIZE 64
#define VS1003_MAX_BUFFER_SIZE 4096
#define DEFAULT_VOLUME 0x2020

#define SCI_MODE    0x00
#define SCI_CLOCKF  0x03
#define SCI_AUDATA  0x05
#define SCI_VOL     0x0B

#define SM_RESET    0x04
#define SM_SDINEW   0x0800

#define GPIO_LABEL_XDCS "myspi-xdcs"
#define GPIO_LABEL_DREQ "myspi-dreq"
#define GPIO_LABEL_XRST "myspi-xrst"

struct vs1003_dev {
    my_spi_slave_data_t slave;
    struct gpio_desc *xdcs_gpio;
    struct gpio_desc *dreq_gpio;
    struct gpio_desc *xrst_gpio;
    struct class *class;
    struct cdev cdev;
    dev_t devt;
    struct mutex lock;
    u8 *stream_buf;
    size_t stream_len, stream_pos;
};

static struct vs1003_dev *vs1003_dev;

static int vs1003_write_sci(struct vs1003_dev *dev, u8 addr, u16 data)
{
    u8 tx_buf[4] = {
        0x02, addr, (data >> 8) & 0xff, data & 0xff
    };
    my_spi_transfer_t xfer = {
        .tx_buf = tx_buf,
        .rx_buf = NULL,
        .len = 4
    };
    return my_spi_sync(&dev->slave, &xfer);
}

static int vs1003_write_stream_chunk(struct vs1003_dev *dev, const u8 *buf, size_t count)
{
    my_spi_transfer_t xfer = {
        .tx_buf = buf,
        .rx_buf = NULL,
        .len = count
    };
    return my_spi_sync(&dev->slave, &xfer);
}

static int vs1003_soft_reset(struct vs1003_dev *dev)
{
    vs1003_write_sci(dev, SCI_MODE, SM_RESET | SM_SDINEW);
    msleep(2);
    vs1003_write_sci(dev, SCI_CLOCKF, 0x6000);
    vs1003_write_sci(dev, SCI_AUDATA, 0x1F40);
    vs1003_write_sci(dev, SCI_VOL, DEFAULT_VOLUME);
    return 0;
}

static ssize_t vs1003_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    if (count > VS1003_MAX_BUFFER_SIZE)
        return -EINVAL;

    if (!vs1003_dev->stream_buf) {
        vs1003_dev->stream_buf = kmalloc(VS1003_MAX_BUFFER_SIZE, GFP_KERNEL);
        if (!vs1003_dev->stream_buf)
            return -ENOMEM;
    }

    if (copy_from_user(vs1003_dev->stream_buf, buf, count))
        return -EFAULT;

    vs1003_dev->stream_len = count;
    vs1003_dev->stream_pos = 0;

    while (vs1003_dev->stream_pos < vs1003_dev->stream_len) {
        size_t remain = vs1003_dev->stream_len - vs1003_dev->stream_pos;
        size_t to_write = min(remain, (size_t)VS1003_CHUNK_SIZE);

        if (vs1003_write_stream_chunk(vs1003_dev, vs1003_dev->stream_buf + vs1003_dev->stream_pos, to_write) < 0)
            break;

        vs1003_dev->stream_pos += to_write;
    }

    return vs1003_dev->stream_pos;
}

static int vs1003_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&vs1003_dev->lock))
        return -EBUSY;
    return vs1003_soft_reset(vs1003_dev);
}

static int vs1003_release(struct inode *inode, struct file *file)
{
    if (vs1003_dev->stream_buf) {
        kfree(vs1003_dev->stream_buf);
        vs1003_dev->stream_buf = NULL;
    }
    mutex_unlock(&vs1003_dev->lock);
    return 0;
}

static const struct file_operations vs1003_fops = {
    .owner = THIS_MODULE,
    .open = vs1003_open,
    .write = vs1003_write,
    .release = vs1003_release,
};

static const struct of_device_id vs1003_of_match[] = {
    { .compatible = "chan,vs1003" },
    { }
};
MODULE_DEVICE_TABLE(of, vs1003_of_match);

static int __init vs1003_init(void)
{
    int ret;
    struct device *dev;
    struct device_node *np = of_find_compatible_node(NULL, NULL, "chan,vs1003");

    vs1003_dev = kzalloc(sizeof(*vs1003_dev), GFP_KERNEL);
    if (!vs1003_dev)
        return -ENOMEM;

    mutex_init(&vs1003_dev->lock);

    my_spi_slave_init(&vs1003_dev->slave);
    vs1003_dev->slave.mode = MYSPI_MODE_0;

    ret = my_spi_register(&vs1003_dev->slave);
    if (ret != SPI_ERR_NONE) {
        pr_err("Failed to register SPI slave\n");
        return -EIO;
    }

    if (np) {
        int gpio;

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

    alloc_chrdev_region(&vs1003_dev->devt, 0, 1, DEV_NAME);
    cdev_init(&vs1003_dev->cdev, &vs1003_fops);
    cdev_add(&vs1003_dev->cdev, vs1003_dev->devt, 1);

    vs1003_dev->class = class_create(CLASS_NAME);
    dev = device_create(vs1003_dev->class, NULL, vs1003_dev->devt, NULL, DEV_NAME);

    pr_info("VS1003 driver initialized\n");
    return 0;
}

static void __exit vs1003_exit(void)
{
    pr_info("VS1003 driver exit\n");

    my_spi_unregister(&vs1003_dev->slave);

    if (vs1003_dev->xdcs_gpio)
        gpiod_put(vs1003_dev->xdcs_gpio);
    if (vs1003_dev->dreq_gpio)
        gpiod_put(vs1003_dev->dreq_gpio);
    if (vs1003_dev->xrst_gpio)
        gpiod_put(vs1003_dev->xrst_gpio);

    device_destroy(vs1003_dev->class, vs1003_dev->devt);
    class_destroy(vs1003_dev->class);
    cdev_del(&vs1003_dev->cdev);
    unregister_chrdev_region(vs1003_dev->devt, 1);

    kfree(vs1003_dev);
    vs1003_dev = NULL;
}


module_init(vs1003_init);
module_exit(vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("VS1003 SPI driver using my_spi");