#define pr_fmt(fmt) "[LCD1602_RAW] " fmt

// pr_debuf 출력
#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#include "lcd1602_def.h"
#include "my_i2c.h"


/*
 * LCD1602 Linux myi2c Character Device Driver
 *
 * - myi2c 인터페이스로 LCD1602 모듈 제어

 * Example usage:

    echo "input text" | sudo tee /dev/lcd1602_raw

 */



#define DEV_NAME "lcd1602_raw"

static dev_t dev_num;
static struct cdev lcd1602_cdev;
static struct class *lcd1602_class;

static void lcd_send_raw4bit(uint8_t cmd);
static void lcd_send_command(uint8_t cmd);
static void lcd_send_data(uint8_t data);
static void lcd_move_cursor(uint8_t row, uint8_t column);

static void lcd_init_sequence(void) {

    pr_debug("lcd_init_sequence\n");

    my_i2c_debug();

    // LCD 초기화 시퀀스 (4비트 모드)

    // power on 후 40ms
    msleep(100);

    my_i2c_lock();
    // 3번 8bit mode 상위비트만
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(5);

    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(1);
    
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(1);

    // 4bit mode 진입 상위비트만
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_4BIT);
    msleep(1);

    pr_debug("4bit mode\n");

    // 명령어 전송
    // 상위 + 하위비트 
    lcd_send_command(LCD_CMD_FUNCTION_SET_2LINE);
    lcd_send_command(LCD_CMD_DISPLAY_ON_CURSOR_OFF);
    lcd_send_command(LCD_CMD_ENTRY_MODE_SET);
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    msleep(2);

    lcd_move_cursor(0, 0);

    my_i2c_unlock();
}

static void lcd_init_message(void) {

    pr_debug("lcd_init_message\n");

    const char* line1 = "LCD1602";
    const char* line2 = "Initialized!";


    my_i2c_lock();

    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);

    lcd_move_cursor(0, 0);
    for (int i = 0; line1[i] && i < LCD1602_LINE_WIDTH; i++) {
        lcd_send_data(line1[i]);
    }

    lcd_move_cursor(1, 0);

    for (int i = 0; line2[i] && i < LCD1602_LINE_WIDTH; i++) {
        lcd_send_data(line2[i]);
    }

    my_i2c_unlock();
}


static void lcd_write_nibble(uint8_t nibble, uint8_t control)
{
    int ret = 0;
    uint8_t data[2] = {0};

    // BACKLIGHT 항상 ON
    control |= LCD_BACKLIGHT;

    // data[0]에 E=1 상태의 바이트 저장 (펄스 시작)
    data[0] = (nibble & 0xF0) | control | LCD_ENABLE;
    
    // data[1]에 E=0 상태의 바이트 저장 (펄스 끝)
    data[1] = (nibble & 0xF0) | control;

    // 1. E=1 상태의 바이트를 먼저 전송합니다.
    ret = my_i2c_write_byte(LCD_ADDR, data[0]);
    if (ret != I2C_ERR_NONE) {
        pr_err("I2C write failed for E=HIGH: data=0x%02X, ret=%d\n", data[0], ret);
        return; 
    }
    
    // E 펄스 폭을 위한 짧은 지연
    udelay(WRITE_TERM_US);

    // 3. E=0 상태의 바이트를 전송합니다.
    ret = my_i2c_write_byte(LCD_ADDR, data[1]);
    if (ret != I2C_ERR_NONE) {
        pr_err("I2C write failed for E=LOW: data=0x%02X, ret=%d\n", data[1], ret);
        return;
    }

    // E 펄스 폭을 위한 짧은 지연
    udelay(WRITE_TERM_US);

}

static void lcd_send_command(uint8_t cmd)
{
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;

    lcd_write_nibble(high, 0x00); // RS=0
    lcd_write_nibble(low, 0x00);
}

static void lcd_send_data(uint8_t data)
{
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;

    // 간헐적인 ack 오류로 delay 추가
    lcd_write_nibble(high, LCD_RS); // RS=1
    udelay(WRITE_TERM_US);
    lcd_write_nibble(low,  LCD_RS);
    udelay(WRITE_TERM_US);
}

// 상위 nibble만 보내는 함수
static void lcd_send_raw4bit(uint8_t cmd)
{
    uint8_t nibble = cmd & 0xF0;  // 상위 4bit만
    lcd_write_nibble(nibble, 0x00); // RS=0, RW=0
}


static void lcd_move_cursor(uint8_t row, uint8_t column)
{
    uint8_t addr = LCD_LINE_ADDR(row, column);
    lcd_send_command(LCD_CMD_SET_DDRAM_ADDR(addr));
}

static int lcd1602_open(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t lcd1602_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
    char *kbuf = 0;
    ssize_t ret = 0;
    size_t max_len = 0, i = 0, col = 0, row = 0;

    pr_debug("lcd1602_write\n");


    if (length == 0) {
        return 0;
    }

    // 최대 32자까지 허용
    max_len = (length > LCD1602_TOTAL_CHARS) ? LCD1602_TOTAL_CHARS : length;

    kbuf = kmalloc(max_len + 1, GFP_KERNEL);
    if (!kbuf) {
        return -ENOMEM;
    }

    if (copy_from_user(kbuf, buffer, max_len)) {
        kfree(kbuf);
        return -EFAULT;
    }

    kbuf[max_len] = '\0';
    ret = max_len;

    // clear
    my_i2c_lock();
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    my_i2c_unlock();

    msleep(2);

    my_i2c_lock();
    lcd_move_cursor(0, 0);

    pr_info("lcd write  : ");
    for (i = 0; i < max_len && row < 2; i++) {
        pr_cont("%c", kbuf[i]);

        if (kbuf[i] == '\n') {
            row++;
            col = 0;
            lcd_move_cursor(row, col);
            continue;
        }

        lcd_send_data(kbuf[i]);
        col++;

        if (col >= LCD1602_LINE_WIDTH) {
            row++;
            col = 0;
            if (row < 2) {
                lcd_move_cursor(row, col);
            }
        }
        msleep(1);
    }
    my_i2c_unlock();


    pr_cont("\n");

    kfree(kbuf);
    return ret;
}

static int lcd1602_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = lcd1602_open,
    .write = lcd1602_write,
    .release = lcd1602_release
};

static int __init lcd1602_init(void)
{
    pr_debug("init\n");

    int ret;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) {
        return ret;
    }

    cdev_init(&lcd1602_cdev, &fops);
    cdev_add(&lcd1602_cdev, dev_num, 1);

    lcd1602_class = class_create(DEV_NAME);
    device_create(lcd1602_class, NULL, dev_num, NULL, DEV_NAME);


    my_i2c_lock();
    ret = my_i2c_register_device(LCD_ADDR, I2C_DEV_LCD1602);
    if(ret != I2C_ERR_NONE) {
        my_i2c_unlock();
        pr_err("Failed to register device on i2c bus: %d\n", ret);
        device_destroy(lcd1602_class, dev_num);
        class_destroy(lcd1602_class);
        cdev_del(&lcd1602_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -EIO;
    }

    ret = my_i2c_ping(LCD_ADDR);
    if(ret != I2C_ERR_NONE) {
        my_i2c_unregister_device(LCD_ADDR);
        my_i2c_unlock();
        pr_err("No device found at address 0x%02X: %d\n", LCD_ADDR, ret);
        device_destroy(lcd1602_class, dev_num);
        class_destroy(lcd1602_class);
        cdev_del(&lcd1602_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -EIO;
    }

    my_i2c_unlock();

    lcd_init_sequence();
    lcd_init_message();

    pr_debug("initialized\n");

    return 0;
}

static void __exit lcd1602_exit(void)
{
    pr_debug("exit\n");

    my_i2c_lock();
    my_i2c_unregister_device(LCD_ADDR);
    my_i2c_unlock();


    device_destroy(lcd1602_class, dev_num);
    class_destroy(lcd1602_class);
    cdev_del(&lcd1602_cdev);
    unregister_chrdev_region(dev_num, 1);
}

module_init(lcd1602_init);
module_exit(lcd1602_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("LCD1602 myi2c Character Device Driver");
