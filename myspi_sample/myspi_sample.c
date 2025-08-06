#define pr_fmt(fmt) "[MYSPI_SAMPLE_RAW] " fmt

// pr_debuf 출력
#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "my_spi.h"

// slave address
#define MYSPI_SAMPLE_CLASS_NAME "myspi_sample_raw"
#define MYSPI_SAMPLE_DEVICE_NAME "myspi_sample"


typedef struct myspi_sample_slave_data{

    my_spi_slave_data_t slave_data;
    struct mutex lock;
}myspi_sample_slave_data_t;

static myspi_sample_slave_data_t* myspi_sample_data;

static struct class* myspi_sample_class;
static struct device* myspi_sample_dev;

static ssize_t pressure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t pressure_show(struct device *dev, struct device_attribute *attr, char *buf);

//static DEVICE_ATTR(pressure);

// attribute 구조체 정의
// show, store 함수 등록
static struct device_attribute dev_attr_pressure = {
    .attr = {
        .name = "pressure",
        .mode  = 0644,
    },
    .show = pressure_show,
    .store = pressure_store,
};

// attribute array 생성
static struct attribute *myspi_sample_attrs[] = {
    &dev_attr_pressure.attr,
    NULL,
};

// grroup화
static const struct attribute_group myspi_sample_group = {
    .attrs = myspi_sample_attrs,
};

// pressure 파일을 읽을 때 호출
static ssize_t pressure_show(struct device *dev, struct device_attribute *attr, char *buf) {
    int pressure_raw = 0;

    pr_debug("pressure_show\n");

    mutex_lock(&myspi_sample_data->lock);

    // 여기에 message 작성
    // 지역변수로 사용해도 될듯
    my_spi_message_t message;
    my_spi_sync((my_spi_slave_data_t*)myspi_sample_data, &message);
    
    mutex_unlock(&myspi_sample_data->lock);

    return sprintf(buf, "%d.%d\n", 0,0);
}


// pressure 파일을 쓸 때 호출됩니다 (OSS 설정).
static ssize_t pressure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int oss_val;

    pr_debug("pressure_store\n");

    if (sscanf(buf, "%d", &oss_val) != 1 || oss_val < 0 || oss_val > 3) {
        return -EINVAL;
    }

    mutex_lock(&myspi_sample_data->lock);


    mutex_unlock(&myspi_sample_data->lock);

    return count;
}


// ----- BMP180 처리 함수 -----

// read 압력 함수
static SPI_error_t myspi_sample_read_pressure(struct myspi_sample_slave_data_t* data, int* out_pressure) {
    SPI_error_t err = SPI_ERR_NONE;
    uint8_t rx[3] = {0};
    uint32_t up = 0;

    pr_debug("myspi_sample_read_pressure\n");

    return SPI_ERR_NONE;
}


// ----- 모듈 init / exit -----
static int __init myspi_sample_raw_init(void)
{
    pr_debug("myspi_sample sysfs init\n");
    SPI_error_t err = SPI_ERR_NONE;

    // class create
    myspi_sample_class = class_create(MYSPI_SAMPLE_CLASS_NAME);
    if (IS_ERR(myspi_sample_class)) {
        pr_err("Failed to create class\n");
        return PTR_ERR(myspi_sample_class);
    }

    // device create
    myspi_sample_dev = device_create(myspi_sample_class, NULL, 0, NULL, MYSPI_SAMPLE_DEVICE_NAME);
    if (IS_ERR(myspi_sample_dev)) {
        pr_err("Failed to create device\n");
        class_destroy(myspi_sample_class);
        return PTR_ERR(myspi_sample_dev);
    }

    // 동적할당
    myspi_sample_data = kzalloc(sizeof(myspi_sample_slave_data_t), GFP_KERNEL);
    if (!myspi_sample_data) {
        pr_err("Failed to allocate myspi_sample_data\n");
        device_destroy(myspi_sample_class, 0);
        class_destroy(myspi_sample_class);
        return -ENOMEM;
    }

    mutex_init(&myspi_sample_data->lock);
    
    my_spi_slave_init((my_spi_slave_data_t*)myspi_sample_data);

    err = my_spi_register((my_spi_slave_data_t*)myspi_sample_data);
    if(err != SPI_ERR_NONE) {
        kfree(myspi_sample_data);
        device_destroy(myspi_sample_class, 0);
        class_destroy(myspi_sample_class);
        pr_err("Failed to register device on spi bus: %d\n", err);
        return -EIO;
    }

    // group으로 한번에 sysfs에 등록
    return sysfs_create_group(&myspi_sample_dev->kobj, &myspi_sample_group);
}

static void __exit myspi_sample_raw_exit(void)
{
    pr_debug("myspi_sample sysfs exit\n");

    my_spi_unregister((my_spi_slave_data_t*)myspi_sample_data);

    if(myspi_sample_data) {
        kfree(myspi_sample_data);
    }

    sysfs_remove_group(&myspi_sample_dev->kobj, &myspi_sample_group);
    device_destroy(myspi_sample_class, 0);
    class_destroy(myspi_sample_class);
}

module_init(myspi_sample_raw_init);
module_exit(myspi_sample_raw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("my spi sample sysfs attribute driver");
