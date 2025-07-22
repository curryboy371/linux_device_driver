#include <linux/module.h>        // module_init, module_exit, MODULE_LICENSE 등
#include <linux/fs.h>            // character device
#include <linux/cdev.h>          // cdev
#include <linux/gpio.h>          // gpio 사용
#include <linux/device.h>        // device 등록

//#include <linux/delay.h>         // udelay

#include <linux/kernel.h> // pr log

#include "my_i2c.h"



/*


 */


static dev_t my_i2c_dev_num;
static struct cdev my_i2c_cdev;
static struct class *my_i2c_class;

// file_operations 구조체
// static const struct file_operations my_i2c_fops = {

// };

// myi2c module init
static int __init my_i2c_init(void)
{
    int ret = 0;

    pr_info("driver init...\n");

    // GPIO 핀 유효성 검사
    if (!gpio_is_valid(I2C_SDA_GPIO) || !gpio_is_valid(I2C_SCL_GPIO)) {
        pr_err("Invalid GPIO pins\n");
        return -EINVAL;
    }

    // GPIO 핀 Request ( Kernel에 pin 사용 요청)
    // 초기값 high ( pull up )
    ret = gpio_request_one(I2C_SDA_GPIO, GPIOF_OUT_INIT_HIGH, "i2c_sda");
    if (ret < 0) {
        pr_err("Failed to request SDA GPIO %d\n", I2C_SDA_GPIO);
        return ret;
    }

    ret = gpio_request_one(I2C_SCL_GPIO, GPIOF_OUT_INIT_HIGH, "i2c_scl");
    if (ret < 0) {
        pr_err("Failed to request SCL GPIO %d\n", I2C_SCL_GPIO);

        gpio_free(I2C_SDA_GPIO); // request된 sda는 해제
        return ret;
    }

    // Character Device 등록 - device 번호 할당
    ret = alloc_chrdev_region(&my_i2c_dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate char device\n");
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // 디바이스 커널에 등록
    cdev_init(&my_i2c_cdev, &my_i2c_fops);
    my_i2c_cdev.owner = THIS_MODULE;

    ret = cdev_add(&my_i2c_cdev, my_i2c_dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add char device\n");
        unregister_chrdev_region(my_i2c_dev_num, 1); // 장치 번호 해제
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // 디바이스의 클래스를 sysfs에 등록 sys/class... (관리 목적)
    // 디바이스가 어떤 그룹에 속하는가
    my_i2c_class = class_create(THIS_MODULE, "my_gpio_i2c_class");
    if (IS_ERR(my_i2c_class)) {
        pr_err("Failed to create device class\n");
        ret = PTR_ERR(my_i2c_class);
        cdev_del(&my_i2c_cdev); // cdev 삭제
        unregister_chrdev_region(my_i2c_dev_num, 1);
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // device file 생성 ( /dev...)
    if (device_create(my_i2c_class, NULL, my_i2c_dev_num, NULL, "my_gpio_i2c_dev") == NULL) {
        pr_err("Failed to create device file\n");
        ret = -ENOMEM;
        class_destroy(my_i2c_class); // 클래스 삭제
        cdev_del(&my_i2c_cdev);
        unregister_chrdev_region(my_i2c_dev_num, 1);
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    pr_info("driver loaded successfully (Major: %d, Minor: %d)\n", MAJOR(my_i2c_dev_num), MINOR(my_i2c_dev_num));
    return 0;
}

// myi2c module exit
static void __exit my_i2c_exit(void)
{
    pr_info("driver exit...\n");

    device_destroy(my_i2c_class, my_i2c_dev_num);
    class_destroy(my_i2c_class);
    cdev_del(&my_i2c_cdev);
    unregister_chrdev_region(my_i2c_dev_num, 1);
    gpio_free(I2C_SCL_GPIO);
    gpio_free(I2C_SDA_GPIO);

    pr_info("driver exit successfully\n");

}

module_init(my_i2c_init);
module_exit(my_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY I2C Character Device Driver");
MODULE_VERSION("0.1");