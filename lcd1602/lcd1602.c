#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

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
 *   echo "Hello,World!" > /dev/lcd1602
 *     => LCD 1줄: Hello
 *              2줄: World!
 */

#define DEV_NAME "lcd1602"
#define LCD_ADDR 0x27
#define BUS_NUM 1

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x00
#define LCD_RS        0x01

#define LCD_CMD_CLEAR_DISPLAY         0x01
#define LCD_CMD_ENTRY_MODE_SET        0x06
#define LCD_CMD_DISPLAY_ON_CURSOR_OFF 0x0C
#define LCD_CMD_FUNCTION_SET_2LINE    0x28
#define LCD_CMD_FUNCTION_SET_8BIT     0x30
#define LCD_CMD_FUNCTION_SET_4BIT     0x20
#define LCD_CMD_SET_DDRAM_ADDR(addr)  (0x80 | ((addr) & 0x7F))

#define LCD_LINE_ADDR(row, col)   (((row)==0 ? 0x00 : 0x40) + (col))
#define LCD1602_LINE_WIDTH 16
#define LCD1602_TOTAL_CHARS (LCD1602_LINE_WIDTH * 2)
#define WRITE_TERM_US 50

static dev_t dev_num;
static struct cdev lcd1602_cdev;
static struct class *lcd1602_class;
static struct i2c_client *lcd_i2c_client;

static void lcd_write_nibble(uint8_t nibble, uint8_t control)
{
    uint8_t data[2];
    control |= LCD_BACKLIGHT;

    data[0] = (nibble & 0xF0) | control | LCD_ENABLE;
    data[1] = (nibble & 0xF0) | control;

    i2c_master_send(lcd_i2c_client, &data[0], 1);
    udelay(10);
    i2c_master_send(lcd_i2c_client, &data[1], 1);
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

static void lcd_init_sequence(void)
{
    msleep(50);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(5);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(1);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_8BIT);
    msleep(1);
    lcd_send_raw4bit(LCD_CMD_FUNCTION_SET_4BIT);
    msleep(1);

    lcd_send_command(LCD_CMD_FUNCTION_SET_2LINE);
    lcd_send_command(LCD_CMD_DISPLAY_ON_CURSOR_OFF);
    lcd_send_command(LCD_CMD_ENTRY_MODE_SET);
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    msleep(2);
    lcd_move_cursor(0, 0);
}

static int lcd1602_open(struct inode *inode, struct file *file) { return 0; }

static ssize_t lcd1602_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
    char *kbuf;
    size_t i;
    uint8_t row = 0, col = 0;

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

    for (i = 0; i < length && row < 2; i++) {
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

static const struct i2c_device_id lcd1602_id[] = {
    { DEV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, lcd1602_id);

static int lcd1602_probe(struct i2c_client *client)
{
    lcd_i2c_client = client;
    lcd_init_sequence();
    return 0;
}

static void lcd1602_remove(struct i2c_client *client) {}

static struct i2c_driver lcd1602_driver = {
    .driver = {
        .name = DEV_NAME,
    },
    .probe = lcd1602_probe,
    .remove = lcd1602_remove,
    .id_table = lcd1602_id,
};

static int __init lcd1602_init(void)
{
    int ret;
    struct i2c_adapter *adapter;
    struct i2c_board_info info = {
        I2C_BOARD_INFO(DEV_NAME, LCD_ADDR)
    };

    adapter = i2c_get_adapter(BUS_NUM);
    if (!adapter)
        return -ENODEV;

    lcd_i2c_client = i2c_new_client_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!lcd_i2c_client)
        return -ENODEV;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) return ret;

    cdev_init(&lcd1602_cdev, &fops);
    cdev_add(&lcd1602_cdev, dev_num, 1);
    lcd1602_class = class_create(DEV_NAME);
    device_create(lcd1602_class, NULL, dev_num, NULL, DEV_NAME);

    return i2c_add_driver(&lcd1602_driver);
}

static void __exit lcd1602_exit(void)
{
    i2c_del_driver(&lcd1602_driver);
    device_destroy(lcd1602_class, dev_num);
    class_destroy(lcd1602_class);
    cdev_del(&lcd1602_cdev);
    unregister_chrdev_region(dev_num, 1);
}

module_init(lcd1602_init);
module_exit(lcd1602_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("LCD1602 Linux I2C Character Device Driver");
