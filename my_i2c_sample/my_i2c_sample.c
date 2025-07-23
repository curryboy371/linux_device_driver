
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


/*
    my_i2c 사용 방법에 대한 i2c device driver smaple
*/

// my_i2c_sample slave address
#define SAMPLE_ADDR 0x23

#define DEVICE_NAME "my_i2c_sample"

// 전역 pointer로 private data 관리
static struct my_i2c_session *my_i2c_session = NULL;


static dev_t my_i2c_sample_dev_num;
static struct cdev my_i2c_sample_cdev;
static struct class* my_i2c_sample_class;

// read

// write


// open
static int my_i2c_sample_open(struct inode *inode, struct file *file) {
    file->private_data = my_i2c_session;  // 고정된 하나의 세션 사용
    return 0;
}


// file_operations 구조체
static const struct file_operations my_i2c_sample_fops = {
    .owner = THIS_MODULE,
    .open = my_i2c_sample_open,
    // .release = my_i2c_release,
    // .read = my_i2c_read,
    // .write = my_i2c_write,
    // .unlocked_ioctl = my_i2c_ioctl,
};

// init
int my_sample_i2c_init(struct file* file) {

    int ret = 0;

    pr_info("driver init...\n");


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

    // private data 할당
    my_i2c_session= kmalloc(sizeof(struct my_i2c_session), GFP_KERNEL);
    if (!my_i2c_session) {
        pr_err("Failed to allocate memory for my_i2c_session\n");
        class_destroy(my_i2c_sample_class); // 클래스 삭제
        cdev_del(&my_i2c_sample_cdev);
        unregister_chrdev_region(my_i2c_sample_dev_num, 1);
        return -ENOMEM;
    }

    my_i2c_session->slave_addr = SAMPLE_ADDR;


    pr_info("driver loaded successfully (Major: %d, Minor: %d)\n", MAJOR(my_i2c_sample_dev_num), MINOR(my_i2c_sample_dev_num));
    return 0;

}


// exit
int my_sample_i2c_exit(struct file* file) {

    pr_info("driver exit...\n");

    // 디바이스 제거
    device_destroy(my_i2c_sample_class, my_i2c_sample_dev_num);

    // 클래스 제거
    class_destroy(my_i2c_sample_class);

    // cdev 제거
    cdev_del(&my_i2c_sample_cdev);

    // 디바이스 번호 해제
    unregister_chrdev_region(my_i2c_sample_dev_num, 1);

    // 세션 메모리 해제
    if (my_i2c_session) {
        kfree(my_i2c_session);
        my_i2c_session = NULL;
    }

    pr_info("driver exit successfully\n");
}


module_init(my_sample_i2c_init);
module_exit(my_sample_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY I2C Sample Character Device Driver");
MODULE_VERSION("0.1");
