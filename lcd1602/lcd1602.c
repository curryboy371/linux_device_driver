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
#define BUS_NUM 3

#define LCD_CMD_CLEAR_DISPLAY         0x01  // 화면 클리어, 커서 홈
#define LCD_CMD_RETURN_HOME           0x02  // 커서 홈으로 이동

#define LCD_CMD_ENTRY_MODE_SET        0x06  // 커서 오른쪽 이동, 화면 이동 없음
#define LCD_CMD_DISPLAY_ON_CURSOR_OFF 0x0C  // 디스플레이 ON, 커서 OFF
#define LCD_CMD_DISPLAY_ON_CURSOR_ON  0x0E  // 디스플레이 ON, 커서 ON
#define LCD_CMD_DISPLAY_ON_BLINK_ON   0x0F  // 디스플레이 ON, 커서 ON + 블링크

#define LCD_CMD_DISPLAY_OFF           0x08  // 디스플레이 OFF, 커서 OFF
#define LCD_CMD_CURSOR_SHIFT_LEFT     0x10  // 커서 왼쪽으로 이동
#define LCD_CMD_CURSOR_SHIFT_RIGHT    0x14  // 커서 오른쪽으로 이동
#define LCD_CMD_DISPLAY_SHIFT_LEFT    0x18  // 화면 왼쪽으로 쉬프트
#define LCD_CMD_DISPLAY_SHIFT_RIGHT   0x1C  // 화면 오른쪽으로 쉬프트

#define LCD_CMD_FUNCTION_SET_8BIT     0x33  // 초기 8비트 인터페이스 설정(초기화 단계)
#define LCD_CMD_FUNCTION_SET_4BIT     0x32  // 4비트 인터페이스 모드로 전환
#define LCD_CMD_FUNCTION_SET_2LINE    0x28  // 4비트 모드, 2라인, 5x8 도트

#define LCD_CMD_SET_CGRAM_ADDR(addr)  (0x40 | ((addr) & 0x3F)) // CGRAM 주소 설정
#define LCD_CMD_SET_DDRAM_ADDR(addr)  (0x80 | ((addr) & 0x7F)) // DDRAM 주소 설정 (커서 위치)


// 0줄이면 0x00 + col
// 1줄이면 0x40 + cal
#define LCD_LINE_ADDR(row, col)   (((row)==0 ? 0x00 : 0x40) + (col))

static dev_t dev_num;
static struct cdev lcd1602_cdev;
static struct class *lcd1602_class;
static struct i2c_client *lcd_i2c_client;

static void lcd_send_command(struct i2c_client *client, uint8_t cmd);
static void lcd_send_data(struct i2c_client *client, uint8_t data);
static void lcd_move_cursor(struct i2c_client *client, uint8_t row, uint8_t column);

static void lcd_init_sequence(struct i2c_client *client)
{
    // LCD 초기화 시퀀스 (4비트 모드)
    msleep(50);
    lcd_send_command(client, LCD_CMD_FUNCTION_SET_8BIT);
    msleep(5);
    lcd_send_command(client, LCD_CMD_FUNCTION_SET_4BIT);
    msleep(5);
    lcd_send_command(client, LCD_CMD_FUNCTION_SET_2LINE);
    lcd_send_command(client, LCD_CMD_DISPLAY_ON_CURSOR_OFF);
    lcd_send_command(client, LCD_CMD_ENTRY_MODE_SET);
    lcd_send_command(client, LCD_CMD_CLEAR_DISPLAY);
    msleep(2);
    lcd_move_cursor(client, 0, 0);
    
}

static void lcd_send_command(struct i2c_client *client, uint8_t cmd)
{
    uint8_t high = cmd & 0xF0;
    uint8_t low = (cmd << 4) & 0xF0;
    uint8_t buf[4] = {
        high | 0x0C, high | 0x08,
        low  | 0x0C, low  | 0x08
    };
    i2c_master_send(client, buf, 4);
}

static void lcd_send_data(struct i2c_client *client, uint8_t data)
{
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    uint8_t buf[4] = {
        high | 0x0D, high | 0x09,
        low  | 0x0D, low  | 0x09
    };
    i2c_master_send(client, buf, 4);
}

static void lcd_move_cursor(struct i2c_client *client, uint8_t row, uint8_t column)
{
    uint8_t addr = LCD_LINE_ADDR(row, column);
    lcd_send_command(client, LCD_CMD_SET_DDRAM_ADDR(addr));
}

static int lcd1602_open(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t lcd1602_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
    char *kbuf;
    ssize_t ret = length;
    size_t i;

    if (length == 0)
        return 0;

    kbuf = kmalloc(length + 1, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    if (copy_from_user(kbuf, buffer, length)) {
        kfree(kbuf);
        return -EFAULT;
    }
    kbuf[length] = '\0';

    // 줄 나누기 
    lcd_move_cursor(lcd_i2c_client, 0, 0);
    for (i = 0; i < length; i++) {
        if (kbuf[i] == ',') {
            lcd_move_cursor(lcd_i2c_client, 1, 0);
            continue;
        }
        lcd_send_data(lcd_i2c_client, kbuf[i]);
        msleep(1);
    }

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

static const struct i2c_device_id lcd1602_id[] = {
    { DEV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, lcd1602_id);

static int lcd1602_probe(struct i2c_client *client)
{
    lcd_i2c_client = client;
    lcd_init_sequence(client);
    return 0;
}

static void lcd1602_remove(struct i2c_client *client)
{
}

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
    if (ret < 0)
        return ret;

    cdev_init(&lcd1602_cdev, &fops);
    cdev_add(&lcd1602_cdev, dev_num, 1);

    lcd1602_class = class_create(DEV_NAME);
    device_create(lcd1602_class, NULL, dev_num, NULL, DEV_NAME);

    ret = i2c_add_driver(&lcd1602_driver);
    if (ret)
        return ret;

    return 0;
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
MODULE_DESCRIPTION("LCD1602 I2C Character Device Driver");
