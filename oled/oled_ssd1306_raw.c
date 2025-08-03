#define pr_fmt(fmt) "[SSD1306_RAW] " fmt
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "my_i2c.h"

#include "oled_font.h"
#include "oled_ssd1306_def.h"


/*
  OLED SSD1306 Linux MyI2C 드라이버 (character driver)
 
  기능 
  -  Custom I2C 를 사용한 i2c 드라이버
  - character driver로 fd를 통한 write
    /dev/oled_ssd1306_raw

  test
    write : echo "abcdefg" | sudo tee /dev/oled_ssd1306_raw

 */

#define DEV_NAME "oled_ssd1306_raw"
#define OLED_ADDR 0x3C

static dev_t dev_num;
static struct cdev oled_cdev;
static struct class *oled_class;

// I2C 1byte Command 전송
static void oled_send_cmd(uint8_t cmd)
{
    i2c_error_t err = I2C_ERR_NONE;
    uint8_t buf[2] = { SSD1306_CO_CMD_SINGLE, cmd };

    err = my_i2c_write_bytes(OLED_ADDR, buf, sizeof(buf), 0);
    if(err != I2C_ERR_NONE) {
        pr_err("write failed: cmd=0x%02X, err=%d\n", cmd, err);
    }
}

// I2C GDDRAM에 픽셀 데이터 연속 전송
// 데이터 길이는 가변적, Stop 조건을 받으면 SSD1306이 데이터 수신을 끝냄
static void oled_send_data(uint8_t *data, size_t len)
{
    uint8_t *buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf) {
        pr_err("Failed to allocate memory for OLED data\n");
        return;
     }

    buf[0] = SSD1306_CO_DATA_SINGLE;
    memcpy(buf + 1, data, len);

    i2c_error_t err = my_i2c_write_bytes(OLED_ADDR, buf, len + 1, 0);
    if(err != I2C_ERR_NONE) {
        pr_err("write failed: data len %zu, err=%d\n", len, err);
    }

    kfree(buf);
}

static void oled_set_pos(uint8_t row, uint8_t col)
{

    oled_send_cmd(SSD1306_CMD_SET_PAGE_START(row));
    oled_send_cmd(SSD1306_CMD_SET_LOW_COLUMN(col));
    oled_send_cmd(SSD1306_CMD_SET_HIGH_COLUMN(col));

    // oled_send_cmd(0xB0 | row);
    // oled_send_cmd(0x00 | (col & 0x0F));
    // oled_send_cmd(0x10 | (col >> 4));
}

static void oled_clear(void)
{
    pr_debug("oled_clear\n");

    uint8_t zero[OLED_WIDTH] = { 0 };
    for (int i = 0; i < OLED_LINE_COUNT; i++) {
        oled_set_pos(i, 0);
        oled_send_data(zero, sizeof(zero));
    }
    oled_set_pos(0, 0);
}

static void oled_display_init_msg(void)
{
    pr_debug("oled_display_init_msg\n");

    oled_clear();
    oled_set_pos(0, 0);

    const char *msg = "SSD1306 Initialized!!!";

    for (size_t i = 0, col = 0; msg[i] != '\0' && col + 6 <= OLED_WIDTH; i++) {
        uint8_t font[6] = {0};
        get_ch_data(msg[i], font);
        oled_send_data(font, sizeof(font));
        col += 6;
    }
}

static void oled_init_sequence(void)
{
    pr_debug("oled_init_sequence\n");

    my_i2c_lock();


    oled_send_cmd(SSD1306_CMD_DISPLAY_OFF);

    oled_send_cmd(SSD1306_CMD_SET_DISPLAY_CLOCK); 
    oled_send_cmd(SSD1306_CLOCK_DIV_DEFAULT);

    oled_send_cmd(SSD1306_CMD_SET_MULTIPLEX);      
    oled_send_cmd(SSD1306_MULTIPLEX_64);

    oled_send_cmd(SSD1306_CMD_SET_DISPLAY_OFFSET); 
    oled_send_cmd(SSD1306_DISPLAY_OFFSET_NONE);

    oled_send_cmd(SSD1306_CMD_SET_START_LINE);

    oled_send_cmd(SSD1306_CMD_CHARGE_PUMP);
    oled_send_cmd(SSD1306_CHARGE_PUMP_ON);

    oled_send_cmd(SSD1306_CMD_MEMORY_MODE);        
    oled_send_cmd(SSD1306_ADDR_MODE_HORIZONTAL);

    oled_send_cmd(SSD1306_CMD_SEG_REMAP);
    oled_send_cmd(SSD1306_CMD_COM_SCAN_DEC);

    oled_send_cmd(SSD1306_CMD_SET_COM_PINS);       
    oled_send_cmd(SSD1306_COM_PINS_ALT_SEQ);

    oled_send_cmd(SSD1306_CMD_SET_CONTRAST);       
    oled_send_cmd(SSD1306_CONTRAST_DEFAULT);

    oled_send_cmd(SSD1306_CMD_SET_PRECHARGE);      
    oled_send_cmd(SSD1306_PRECHARGE_DEFAULT);

    oled_send_cmd(SSD1306_CMD_SET_VCOM_DETECT);    
    oled_send_cmd(SSD1306_VCOM_DESELECT_DEFAULT);

    oled_send_cmd(SSD1306_CMD_DISPLAY_ALL_ON_RESUME);
    oled_send_cmd(SSD1306_CMD_NORMAL_DISPLAY);

    oled_send_cmd(SSD1306_CMD_DISPLAY_ON);

    oled_display_init_msg();

    my_i2c_unlock();

}


static int oled_ssd1306_open(struct inode *inode, struct file *file) { 
    pr_debug("oled_ssd1306_open\n");

    return 0; 
}

static ssize_t oled_ssd1306_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
    char *kbuf;
    size_t i;
    uint8_t row = 0, col = 0;

    pr_debug("oled_ssd1306_write\n");

    if (length == 0) {
        return 0; 
    }


    kbuf = kmalloc(length + 1, GFP_KERNEL);
    if (!kbuf)  {
        return -ENOMEM; 
    }

    if (copy_from_user(kbuf, buffer, length)) {
        kfree(kbuf);
        return -EFAULT;
    }
    kbuf[length] = '\0';

    pr_info("write: %s\n", kbuf);

    my_i2c_lock();

    oled_clear();

    for (i = 0; i < length && row < OLED_LINE_COUNT; i++) {
        if (kbuf[i] == '\n') {
            row++;
            col = 0;
            continue;
        }

        oled_set_pos(row, col);
        uint8_t font[6] = {0};
        get_ch_data(kbuf[i], font);  // extern function
        oled_send_data(font, sizeof(font));
        col += 6;

        if (col + 6 >= OLED_WIDTH) {
            row++;
            col = 0;
        }
    }

    my_i2c_unlock();

    kfree(kbuf);
    return length;
}

static int oled_ssd1306_release(struct inode *inode, struct file *file) { 

    pr_debug("oled_ssd1306_release\n");

    return 0; 
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = oled_ssd1306_open,
    .write = oled_ssd1306_write,
    .release = oled_ssd1306_release
};

static int __init oled_ssd1306_init(void)
{
    int ret;

    pr_debug("init OLED SSD1306 driver.\n");

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0)  {
        return ret;
    }

    cdev_init(&oled_cdev, &fops);
    cdev_add(&oled_cdev, dev_num, 1);
    oled_class = class_create(DEV_NAME);
    device_create(oled_class, NULL, dev_num, NULL, DEV_NAME);


    my_i2c_lock();
    ret = my_i2c_register_device(OLED_ADDR, I2C_DEV_GENERIC);
    if(ret != I2C_ERR_NONE) {
        my_i2c_unlock();
        pr_err("Failed to register device on i2c bus: %d\n", ret);
        device_destroy(oled_class, dev_num);
        class_destroy(oled_class);
        cdev_del(&oled_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -EIO;
    }

    ret = my_i2c_ping(OLED_ADDR);
    if(ret != I2C_ERR_NONE) {
        my_i2c_unregister_device(OLED_ADDR);
        my_i2c_unlock();
        pr_err("No device found at address 0x%02X: %d\n", OLED_ADDR, ret);
        device_destroy(oled_class, dev_num);
        class_destroy(oled_class);
        cdev_del(&oled_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -EIO;
    }

    my_i2c_unlock();


    oled_init_sequence();


    return 0;

}

static void __exit oled_ssd1306_exit(void)
{
    pr_debug("exit OLED SSD1306 driver.\n");

    my_i2c_lock();
    my_i2c_unregister_device(OLED_ADDR);
    my_i2c_unlock();

    device_destroy(oled_class, dev_num);
    class_destroy(oled_class);
    cdev_del(&oled_cdev);
    unregister_chrdev_region(dev_num, 1);
}

module_init(oled_ssd1306_init);
module_exit(oled_ssd1306_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("SSD1306 OLED MyI2C Character Device Driver");