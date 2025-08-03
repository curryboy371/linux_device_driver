#define pr_fmt(fmt) "[LCD1602] " fmt

// pr_debuf 출력
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "lcd1602_def.h"

/*
 * LCD1602 Linux I2C Character Device Driver
 *
 * - I2C 인터페이스로 LCD1602 모듈 제어
 * - BUS_NUM (ex: i2c-3) 어댑터를 사용해 LCD1602를 직접 장치 등록
 * - 문자 출력용 /dev/lcd1602 캐릭터 디바이스 자동 생성 (udev)
 * - echo/write로 문자열 전달 시 LCD에 표시
 *   - ','(콤마) 입력 시 2번째 줄로 커서 이동
 *
 * - ioctl 없이 단순 write() 방식으로 문자 출력
 * - i2c_master_send()로 4비트 모드 명령 전송
 * - 커널 모듈 로딩시 자동으로 LCD 초기화
 *
 * Example usage:
    write : echo "abcdefg" | sudo tee /dev/lcd1602
 */

#define DEV_NAME "lcd1602"


static dev_t dev_num;
static struct cdev lcd1602_cdev;
static struct class *lcd1602_class;
static struct i2c_client *lcd_i2c_client;

static void lcd_write_nibble(uint8_t nibble, uint8_t control)
{
    uint8_t data[2];


    if(!lcd_i2c_client) {
        pr_err("lcd_i2c_client is null\n");
        return;
    }

    control |= LCD_BACKLIGHT;

    data[0] = (nibble & 0xF0) | control | LCD_ENABLE;
    data[1] = (nibble & 0xF0) | control;

    int ret = i2c_master_send(lcd_i2c_client, &data[0], 1);
    if (ret != 1) {
        pr_err("send failed at EN=1: ret=%d\n", ret);
    }
    udelay(WRITE_TERM_US);

    ret = i2c_master_send(lcd_i2c_client, &data[1], 1);
    if (ret != 1) {
        pr_err("send failed at EN=0: ret=%d\n", ret);
    }
    udelay(WRITE_TERM_US);
}

static void lcd_send_raw4bit(uint8_t cmd)
{
    lcd_write_nibble(cmd & 0xF0, 0x00);
}

static void lcd_send_command(uint8_t cmd)
{
    lcd_write_nibble(cmd & 0xF0, 0x00);
    lcd_write_nibble((cmd << 4) & 0xF0, 0x00);
}

static void lcd_send_data(uint8_t data)
{
    lcd_write_nibble(data & 0xF0, LCD_RS);
    udelay(WRITE_TERM_US);
    lcd_write_nibble((data << 4) & 0xF0, LCD_RS);
    udelay(WRITE_TERM_US);
}

static void lcd_move_cursor(uint8_t row, uint8_t column)
{
    lcd_send_command(LCD_CMD_SET_DDRAM_ADDR(LCD_LINE_ADDR(row, column)));
}


static void lcd_init_message(void) {

    pr_debug("lcd_init_message\n");

    const char* line1 = "LCD1602 I2C";
    const char* line2 = "Initialized!";

    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);

    lcd_move_cursor(0, 0);
    for (int i = 0; line1[i] && i < LCD1602_LINE_WIDTH; i++) {
        lcd_send_data(line1[i]);
    }

    lcd_move_cursor(1, 0);

    for (int i = 0; line2[i] && i < LCD1602_LINE_WIDTH; i++) {
        lcd_send_data(line2[i]);
    }
}



static void lcd_init_sequence(void)
{
    pr_debug("lcd_init_sequence\n");
    msleep(100);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(5);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(2);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(2);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_4BIT);
    msleep(2);

    pr_debug("4bit mode\n");

    lcd_send_command(LCD_CMD_FUNCTION_SET_2LINE);
    lcd_send_command(LCD_CMD_DISPLAY_ON_CURSOR_OFF);
    lcd_send_command(LCD_CMD_ENTRY_MODE_SET);
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    msleep(5);
    lcd_move_cursor(0, 0);
}

static int lcd1602_open(struct inode *inode, struct file *file) { return 0; }

static ssize_t lcd1602_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
    char *kbuf;
    size_t i;
    uint8_t row = 0, col = 0;

    pr_info("lcd1602_write\n");

    if (length == 0) return 0;

    kbuf = kmalloc(length + 1, GFP_KERNEL);
    if (!kbuf) return -ENOMEM;

    if (copy_from_user(kbuf, buffer, length)) {
        kfree(kbuf);
        return -EFAULT;
    }
    kbuf[length] = '\0';

    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    msleep(2);
    lcd_move_cursor(0, 0);

    pr_info("lcd write  : ");

    for (i = 0; i < length && row < 2; i++) {
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
            if (row < 2)
                lcd_move_cursor(row, col);
        }
        msleep(1);
    }
    pr_cont("\n");

    kfree(kbuf);
    return length;
}

static int lcd1602_release(struct inode *inode, struct file *file) { return 0; }

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = lcd1602_open,
    .write = lcd1602_write,
    .release = lcd1602_release
};

static int lcd1602_probe(struct i2c_client *client)
{
    int ret;
    lcd_i2c_client = client;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) return ret;

    cdev_init(&lcd1602_cdev, &fops);
    ret = cdev_add(&lcd1602_cdev, dev_num, 1);
    if (ret < 0) goto err_cdev_add;

    lcd1602_class = class_create(DEV_NAME);
    if (IS_ERR(lcd1602_class)) {
        ret = PTR_ERR(lcd1602_class);
        goto err_class_create;
    }

    if (IS_ERR(device_create(lcd1602_class, NULL, dev_num, NULL, DEV_NAME))) {
        ret = -ENODEV;
        goto err_device_create;
    }

    lcd_init_sequence();
    msleep(50);
    lcd_init_message();
    pr_info("LCD1602 Driver probed successfully\n");
    return 0;

err_device_create:
    class_destroy(lcd1602_class);
err_class_create:
    cdev_del(&lcd1602_cdev);
err_cdev_add:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void lcd1602_remove(struct i2c_client *client)
{
    device_destroy(lcd1602_class, dev_num);
    class_destroy(lcd1602_class);
    cdev_del(&lcd1602_cdev);
    unregister_chrdev_region(dev_num, 1);
    lcd_i2c_client = NULL;
}

static const struct of_device_id lcd_of_match[] = {
    { .compatible = "i2c-lcd,lcd1602" },
    { }
};
MODULE_DEVICE_TABLE(of, lcd_of_match);

static struct i2c_driver lcd1602_driver = {
    .driver = {
        .name = DEV_NAME,
        .of_match_table = lcd_of_match,
    },
    .probe = lcd1602_probe,
    .remove = lcd1602_remove,
};


static int __init lcd1602_init(void)
{
    pr_info("init driver.\n");
    return i2c_add_driver(&lcd1602_driver);
}

static void __exit lcd1602_exit(void)
{
    pr_info("exit driver.\n");
    i2c_del_driver(&lcd1602_driver);
}

module_init(lcd1602_init);
module_exit(lcd1602_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("LCD1602 Linux I2C Character Device Driver");
