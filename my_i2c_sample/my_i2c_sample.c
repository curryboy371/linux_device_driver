
#define pr_fmt(fmt) "[my_i2c_sample] " fmt


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kernel.h>


#include "my_i2c.h"
#include "my_i2c_sample.h"

/*
    my_i2c 사용 방법에 대한 i2c device driver smaple
*/

// my_i2c_sample slave address
#define SAMPLE_ADDR 0x77
#define MAX_BUF_SIZE 128

#define DEVICE_NAME "my_i2c_sample"


static dev_t my_i2c_sample_dev_num;
static struct cdev my_i2c_sample_cdev;
static struct class* my_i2c_sample_class;


static const int OSS_WAIT_TIME_MS[4] = { 5, 8, 14, 26 };

// temp 보정값을 적용하여 반환
static s32 bmp180_calc_temperature(struct bmp180_data* data, s32 ut)
{
    s32 x1, x2, t;

    x1 = ((ut - data->calib.AC6) * data->calib.AC5) >> 15;
    x2 = (data->calib.MC << 11) / (x1 + data->calib.MD);

    data->calib.B5 = x1 + x2;
    t = (data->calib.B5 + 8) >> 4;

    return t;
}

// pressure 보정값을 적용하여 반환
static s32 bmp180_calc_pressure(struct bmp180_data* data, s32 up)
{
    s32 b6, x1, x2, x3, b3;
    u32 b4, b7;
    s32 p;

    b6 = data->calib.B5 - 4000;

    x1 = (data->calib.B2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (data->calib.AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((data->calib.AC1 * 4 + x3) << data->oss) + 2) >> 2;

    x1 = (data->calib.AC3 * b6) >> 13;
    x2 = (data->calib.B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;

    b4 = (data->calib.AC4 * (u32)(x3 + 32768)) >> 15;
    b7 = ((u32)up - b3) * (50000 >> data->oss);

    if (b7 < 0x80000000)
        p = (b7 * 2) / b4;
    else
        p = (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;

    p = p + ((x1 + x2 + 3791) >> 4);

    return p;
}

// calb 테이블을 설정하기 위해 register를 읽어 table을 저장하는 함수
static int bmp180_read_calib(struct bmp180_data* data, struct bmp180_calib* calib) {

    pr_info("Calibration read start\n");

    uint8_t raw[CALIB_DATA_LENGTH];
    
    int ret = my_i2c_write_bytes(data->slave_addr, CALIB_DATA_START);
    if (ret != I2C_ERR_NONE) { 
        return -EIO;
    }

    ret = my_i2c_read_bytes(data->slave_addr, raw, CALIB_DATA_LENGTH);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    int16_t* table[] = {
        &calib->AC1, &calib->AC2, &calib->AC3,
        (int16_t*)&calib->AC4, (int16_t*)&calib->AC5, (int16_t*)&calib->AC6,
        &calib->B1, &calib->B2,
        &calib->MB, &calib->MC, &calib->MD
    };

    for (int i = 0; i < CALIB_DATA_LENGTH / 2; i++) {
        *table[i] = (int16_t)(raw[2 * i] << 8 | raw[2 * i + 1]);
    }

    return 0;
}




// 온도 읽기
static int bmp180_read_temperature(struct bmp180_data* data, int32_t* out_temp) {

    uint8_t temp_cmd[2] = { CTRL_MEAS, TEMP };
    uint8_t rx[2];
    int ret;

    ret = my_i2c_write_bytes(data->slave_addr, temp_cmd, 2);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    msleep(OSS_WAIT_TIME_MS[0]);

    // 측정 결과 MSB 부터 read

    ret = my_i2c_write_byte(data->slave_addr, OUT_MSB);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }


    ret = my_i2c_read_bytes(data->slave_addr, rx, 2);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    // 측정 값 보정 연산
    bmp180_calc_temperature();
    int32_t ut = (rx[0] << 8) | rx[1];
    int32_t x1 = ((ut - data->calib.AC6) * data->calib.AC5) >> 15;
    int32_t x2 = (data->calib.MC << 11) / (x1 + data->calib.MD);
    data->calib.B5 = x1 + x2;
    *out_temp = (data->calib.B5 + 8) >> 4;

    return 0;
}

// 압력 읽기
static int bmp180_read_pressure(struct bmp180_data* data, int32_t* out_pressure) {


    uint8_t ctrl_value = PRESSURE | (data->oss << 6);
    uint8_t cmd[2] = { CTRL_MEAS, ctrl_value };
    uint8_t rx[3];
    int ret;

    // 측정 명령
    ret = my_i2c_write_bytes(data->slave_addr, cmd, 2);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    msleep(OSS_WAIT_TIME_MS[data->oss]);

    // 측정 read, MSB부터 read
    ret = my_i2c_write_byte(data->slave_addr, OUT_MSB);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    ret = my_i2c_read_bytes(data->slave_addr, rx, 3);
    if (ret != I2C_ERR_NONE) {
        return -EIO;
    }

    // uncompensated pressure 계산
    s32 u_pressure = ((rx_data[BMP_MSB] << 16) | (rx_data[BMP_LSB] << 8) | rx_data[BMP_XLSB] >> (8 - data->oss));

    *out_pressure = bmp180_calc_pressure(data, u_pressure);

	return 0;
}


// read
static ssize_t my_i2c_sample_read(struct file* file, char __user* buf, size_t count, loff_t* ppos) {
    struct my_i2c_session* sess = file->private_data;

    if (!sess) {
        pr_err("Session is NULL\n");
        return -EINVAL;
    }

    return 1;
}

// write


// open
static int my_i2c_sample_open(struct inode *inode, struct file *file) {

    pr_info("my_i2c_sample open called\n");

    my_i2c_init_gpio();
    my_i2c_debug();

    struct my_i2c_session *sess;

    sess = kmalloc(sizeof(*sess), GFP_KERNEL);
    if (!sess) {
        pr_err("Failed to allocate memory for session\n");
        return -ENOMEM;
    }

    sess->slave_addr = SAMPLE_ADDR;
    file->private_data = sess;

    my_i2c_ping(sess->slave_addr);

    pr_info("my_i2c_sample session created with slave address: 0x%02X\n", sess->slave_addr);
    return 0;
}

static ssize_t my_i2c_sample_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct my_i2c_session *sess = file->private_data;
    uint8_t kbuf[MAX_BUF_SIZE];
    size_t bytes_written = 0;

    if (!sess) {
        pr_err("Session is NULL\n");
        return -EINVAL;
    }

    if (count > MAX_BUF_SIZE) {
        pr_err("Write size too big\n");
        return -EINVAL;
    }

    if (copy_from_user(kbuf, buf, count)) {
        pr_err("Failed to copy data from user\n");
        return -EFAULT;
    }


    return bytes_written;
}

// release
static int my_i2c_sample_release(struct inode *inode, struct file *file) {
    struct my_i2c_session *sess = file->private_data;
    
    pr_info("my_i2c_sample release called\n");
    if(sess) {
        kfree(sess);
    }

    return 0;
}

// file_operations 구조체
static const struct file_operations my_i2c_sample_fops = {
    .owner = THIS_MODULE,
    .open = my_i2c_sample_open,
    .release = my_i2c_sample_release,
    .read = my_i2c_sample_read,
    .write = my_i2c_sample_write,
};

// init
static int __init my_sample_i2c_init(void) {

    int ret = 0;

    pr_info("my_i2c_sample driver init\n");

    // Character Device 등록 - device 번호 할당
    ret = alloc_chrdev_region(&my_i2c_sample_dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate char device\n");
        return ret;
    }

    // 디바이스 커널에 등록
    cdev_init(&my_i2c_sample_cdev, &my_i2c_sample_fops);
    my_i2c_sample_cdev.owner = THIS_MODULE;

    ret = cdev_add(&my_i2c_sample_cdev, my_i2c_sample_dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add char device\n");
        unregister_chrdev_region(my_i2c_sample_dev_num, 1); // 장치 번호 해제
        return ret;
    }

    // 디바이스의 클래스를 sysfs에 등록 sys/class... (관리 목적)
    // 디바이스가 어떤 그룹에 속하는가
    my_i2c_sample_class = class_create("my_i2c_sample_class");
    if (IS_ERR(my_i2c_sample_class)) {
        pr_err("Failed to create device class\n");
        ret = PTR_ERR(my_i2c_sample_class);
        cdev_del(&my_i2c_sample_cdev); // cdev 삭제
        unregister_chrdev_region(my_i2c_sample_dev_num, 1);
        return ret;
    }

    // device file 생성 ( /dev...)
    if (device_create(my_i2c_sample_class, NULL, my_i2c_sample_dev_num, NULL, "my_i2c_sample_dev") == NULL) {
        pr_err("Failed to create device file\n");
        ret = -ENOMEM;
        class_destroy(my_i2c_sample_class); // 클래스 삭제
        cdev_del(&my_i2c_sample_cdev);
        unregister_chrdev_region(my_i2c_sample_dev_num, 1);
        return ret;
    }

    pr_info("driver loaded successfully (Major: %d, Minor: %d)\n", MAJOR(my_i2c_sample_dev_num), MINOR(my_i2c_sample_dev_num));
    return 0;

}

// exit
static void __exit my_sample_i2c_exit(void) {


    pr_info("my_i2c_sample driver exit\n");

    // 디바이스 제거
    device_destroy(my_i2c_sample_class, my_i2c_sample_dev_num);

    // 클래스 제거
    class_destroy(my_i2c_sample_class);

    // cdev 제거
    cdev_del(&my_i2c_sample_cdev);

    // 디바이스 번호 해제
    unregister_chrdev_region(my_i2c_sample_dev_num, 1);

    pr_info("my_i2c_sample driver removed\n");
}


module_init(my_sample_i2c_init);
module_exit(my_sample_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY I2C Sample Character Device Driver");
