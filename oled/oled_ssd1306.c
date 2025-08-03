#define pr_fmt(fmt) "[SSD1306] " fmt
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

#include "oled_font.h"
#include "oled_ssd1306_def.h"


/*
  OLED SSD1306 Linux I2C 드라이버 (character driver)
 
  기능 
  -  linux i2c clinet를 사용한 i2c 클라이언트 driver 
  - character driver로 fd를 통한 write
    /dev/oled_ssd1306

  test
    write : echo "abcdefg" | sudo tee /dev/oled_ssd1306

 */


#define DEV_NAME "oled_ssd1306"
#define OLED_ADDR 0x3C
#define BUS_NUM 1


static dev_t dev_num;
static struct cdev oled_cdev;
static struct class *oled_class;
static struct i2c_client *oled_client;

// I2C 1byte Command 전송
static void oled_send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CO_CMD_SINGLE, cmd };

    i2c_master_send(oled_client, buf, 2);
}

// I2C GDDRAM에 픽셀 데이터 연속 전송
// 데이터 길이는 가변적, Stop 조건을 받으면 SSD1306이 데이터 수신을 끝냄
static void oled_send_data(uint8_t *data, size_t len)
{
    uint8_t *buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf) {
        pr_err("Failed to allocate memory for data buffer\n");
        return;
    }

    if(!oled_client) {
        pr_err("oled_client is null\n");
        kfree(buf);
        return;
    }

    buf[0] = SSD1306_CO_DATA_SINGLE;
    memcpy(buf + 1, data, len);
    i2c_master_send(oled_client, buf, len + 1);
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


// DTS compatible 문자열과 일치하도록 of_device_id 테이블을 추가
// dtbo로 overlay
static const struct of_device_id oled_of_match[] = {
    { .compatible = "i2c-oled,ssd1306", },
    { }
};
MODULE_DEVICE_TABLE(of, oled_of_match);

static int oled_ssd1306_probe(struct i2c_client *client)
{
    pr_debug("oled_ssd1306_probe\n");

    int ret = 0;
    oled_client = client;

    // 문자 디바이스 번호 할당
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("Failed to alloc_chrdev_region\n");
        return ret;
    }

    // cdev 구조체 초기화
    cdev_init(&oled_cdev, &fops);

    // cdev 커널에 등록
    ret = cdev_add(&oled_cdev, dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to cdev_add\n");
        goto err_cdev_add;
    }

    // 디바이스 클래스 생성
    oled_class = class_create(DEV_NAME);
    if (IS_ERR(oled_class)) {
        pr_err("Failed to class_create\n");
        ret = PTR_ERR(oled_class);
        goto err_class_create;
    }

    // 5단계: 디바이스 파일 생성
    if (IS_ERR(device_create(oled_class, NULL, dev_num, NULL, DEV_NAME))) {
        pr_err("Failed to device_create\n");
        ret = -ENODEV;
        goto err_device_create;
    }

    oled_init_sequence(); // OLED 초기화 시퀀스 실행
    pr_info("OLED SSD1306 Driver probed successfully\n");

    return 0;


err_device_create:
    class_destroy(oled_class);
err_class_create:
    cdev_del(&oled_cdev);
err_cdev_add:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void oled_ssd1306_remove(struct i2c_client *client) {

    pr_info("oled_ssd1306_remove is called\n");
    device_destroy(oled_class, dev_num);
    class_destroy(oled_class);
    cdev_del(&oled_cdev);
    unregister_chrdev_region(dev_num, 1);
    oled_client = NULL;
}

static struct i2c_driver oled_driver = {
    .driver = {
        .name = DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = oled_of_match, // dts 매칭 테이블 추가
    },
    .probe = oled_ssd1306_probe,
    .remove = oled_ssd1306_remove,
};

static int __init oled_ssd1306_init(void)
{

    pr_info("init OLED SSD1306 driver.\n");
    return i2c_add_driver(&oled_driver);
}

static void __exit oled_ssd1306_exit(void)
{
    pr_info("exit OLED SSD1306 driver.\n");
    i2c_del_driver(&oled_driver);
}

module_init(oled_ssd1306_init);
module_exit(oled_ssd1306_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("SSD1306 OLED I2C Character Device Driver");
