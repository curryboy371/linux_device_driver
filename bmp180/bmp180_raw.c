#define pr_fmt(fmt) "[BMP180_RAW] " fmt

// pr_debuf 출력
#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "bmp180_def.h"

#include "my_i2c.h"

/*
  BMP180 Linux I2C 드라이버 (sysfs + my i2c flatform driver)
 
  - i2c_driver 구조체를 사용한 Linux I2C 클라이언트 드라이버
  - 사용자 공간과 sysfs를 통해 직접 값 읽기/쓰기 가능:
    /sys/class/bmp180_raw/bmp180/temperature
    /sys/class/bmp180_raw/bmp180/pressure
    cat, echo로 직접 측정값 확인 및 설정 가능.


    test
    oos write : echo 2 | sudo tee /sys/class/bmp180_raw/bmp180/pressure
    temp read : sudo cat /sys/class/bmp180_raw/bmp180/temperature
    sudo cat /sys/class/bmp180_raw/bmp180/pressure

 */

// slave address
#define BMP180_CLASS_NAME "bmp180_raw"
#define BMP180_DEVICE_NAME "bmp180"


// bmp180 data 
struct bmp180_data {
    uint8_t slave_addr;          
    uint8_t oss;                 // OverSampling Setting (0~3)
    struct bmp180_calib calib;  
    struct mutex lock;
    uint8_t is_calib_read; // 캘리브레이션 데이터 로드 여부
};


static struct class* bmp_class;
static struct device* bmp_dev;
static struct bmp180_data* bmp_data;

static const int OSS_WAIT_TIME_MS[4] = { 5, 8, 14, 26 };


static i2c_error_t bmp180_read_pressure(struct bmp180_data* data, int* out_pressure);
static i2c_error_t bmp180_read_calib(struct bmp180_data *data);

static i2c_error_t bmp180_read_temperature(struct bmp180_data *data, int *temp_out);


static s32 bmp180_calc_b5(struct bmp180_calib *calib, u16 ut);
static int bmp180_calc_pressure(struct bmp180_data *data, uint32_t up);
static ssize_t pressure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t pressure_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t temperature_show(struct device *dev, struct device_attribute *attr, char *buf);


// linux/device.h
//static DEVICE_ATTR(temperature);
//static DEVICE_ATTR(pressure);

// attribute 구조체 정의
// show, store 함수 등록
static struct device_attribute dev_attr_temperature = {
    .attr = {
        .name = "temperature",
        .mode  = 0644,
    },
    .show = temperature_show,
};

static struct device_attribute dev_attr_pressure = {
    .attr = {
        .name = "pressure",
        .mode  = 0644,
    },
    .show = pressure_show,
    .store = pressure_store,
};

// attribute array 생성
static struct attribute *bmp_attrs[] = {
    &dev_attr_temperature.attr,
    &dev_attr_pressure.attr,
    NULL,
};

// grroup화
static const struct attribute_group bmp_group = {
    .attrs = bmp_attrs,
};



// ----- Sysfs show/store 함수 구현 -----

// temperature 파일을 읽음
static ssize_t temperature_show(struct device *dev, struct device_attribute *attr, char *buf) {
    int temp_raw = 0;
    i2c_error_t err = I2C_ERR_NONE;

    pr_debug("temperature_show\n");


    mutex_lock(&bmp_data->lock);

    // 온도 측정
    my_i2c_lock();
    err = bmp180_read_temperature(bmp_data, &temp_raw);
    my_i2c_unlock();

    mutex_unlock(&bmp_data->lock);

    if (err != I2C_ERR_NONE) {
        return err;
    }

    return sprintf(buf, "%d.%d\n", temp_raw / 10, temp_raw % 10);
}

// pressure 파일을 읽을 때 호출
static ssize_t pressure_show(struct device *dev, struct device_attribute *attr, char *buf) {
    int pressure_raw = 0;
    int temp_raw = 0;
    i2c_error_t err;

    pr_debug("pressure_show\n");

    mutex_lock(&bmp_data->lock);

    // 압력 계산을 위한 온도 측정
    my_i2c_lock();
    err = bmp180_read_temperature(bmp_data, &temp_raw);
    if (err == I2C_ERR_NONE) {
        err = bmp180_read_pressure(bmp_data, &pressure_raw);
    }
    my_i2c_unlock();

    mutex_unlock(&bmp_data->lock);

    if (err != I2C_ERR_NONE) {
        return err;
    }
    return sprintf(buf, "%d.%d\n", pressure_raw / 100, pressure_raw % 100);
}


// pressure 파일을 쓸 때 호출됩니다 (OSS 설정).
static ssize_t pressure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int oss_val;

    pr_debug("pressure_store\n");

    if (sscanf(buf, "%d", &oss_val) != 1 || oss_val < 0 || oss_val > 3) {
        return -EINVAL;
    }

    mutex_lock(&bmp_data->lock);
    bmp_data->oss = oss_val;
    mutex_unlock(&bmp_data->lock);

    pr_debug("BMP180 Oversampling Setting (OSS) updated to %d\n", bmp_data->oss);

    return count;
}


// ----- BMP180 처리 함수 -----

// read 테이블 함수
static i2c_error_t bmp180_read_calib(struct bmp180_data *data)
{
    if (data->is_calib_read) {
        return I2C_ERR_NONE;
    }

    pr_debug("read calib table\n");


    i2c_error_t err = I2C_ERR_NONE;

    uint8_t raw[CALIB_DATA_LENGTH];
    int16_t *table[] = {
        &data->calib.AC1, &data->calib.AC2, &data->calib.AC3,
        (int16_t *)&data->calib.AC4, (int16_t *)&data->calib.AC5, (int16_t *)&data->calib.AC6,
        &data->calib.B1, &data->calib.B2,
        &data->calib.MB, &data->calib.MC, &data->calib.MD
    };

    // slave 주소에서 calib start reg을 22byte만큼 read
    err = my_i2c_read_reg_bytes(data->slave_addr, CALIB_DATA_START, raw, sizeof(raw));
    if (err != I2C_ERR_NONE) {
        my_i2c_unlock();
        return err;
    }

    for (int i = 0; i < 11; ++i) {
        *table[i] = (int16_t)(raw[2 * i] << 8 | raw[2 * i + 1]);
    }

    data->is_calib_read = 1;

    pr_debug("read calib table successpully\n");

    return err;
}

// read 압력 함수
static i2c_error_t bmp180_read_pressure(struct bmp180_data* data, int* out_pressure) {
    i2c_error_t err = I2C_ERR_NONE;
    uint8_t rx[3] = {0};
    uint32_t up = 0;

    pr_debug("bmp180_read_pressure\n");


    err = bmp180_read_calib(data);
    if (err != I2C_ERR_NONE) {
        pr_err("calib table read failed\n");
        return err;
    }

    // 측정 명령 전송
    uint8_t ctrl = CTRL_MEAS;
    uint8_t cmd = PRESSURE | (data->oss << 6);
    uint8_t write_buf[2] = { ctrl, cmd };

    err = my_i2c_write_bytes(data->slave_addr, write_buf, sizeof(write_buf), 0);
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to write pressure measure command\n");
        return err;
    }

    // 측정 시간 대기
    msleep(OSS_WAIT_TIME_MS[data->oss]);

    // 결과 주소 설정
    err = my_i2c_write_byte(data->slave_addr, OUT_MSB);
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to set OUT_MSB address\n");
        return err;
    }

    // 결과 읽기 (3바이트)
    err = my_i2c_read_bytes(data->slave_addr, rx, sizeof(rx));
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to read pressure result\n");
        return err;
    }

    // uncompensated pressure 계산
    up = ((rx[0] << 16) | (rx[1] << 8) | rx[2]) >> (8 - data->oss);

    // 보정된 압력 계산
    *out_pressure = bmp180_calc_pressure(data, up);

    return I2C_ERR_NONE;
}

// read 온도 함수
static i2c_error_t bmp180_read_temperature(struct bmp180_data *data, int *temp_out) {

    i2c_error_t err = I2C_ERR_NONE;

    uint8_t rx[2] = {0};
    int32_t ut = 0;

    pr_debug("bmp180_read_temperature\n");


    err = bmp180_read_calib(data);
    if (err != I2C_ERR_NONE) {
        pr_err("calib table read failed\n");
        return err;
    }

    // 온도 측정 명령 전송
    uint8_t cmd[2] = { CTRL_MEAS, TEMP };
    err = my_i2c_write_bytes(data->slave_addr, cmd, sizeof(cmd), 0);
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to write TEMP_CMD\n");
        return err;
    }

    msleep(OSS_WAIT_TIME_MS[0]);

    // 결과 레지스터 주소 설정
    err = my_i2c_write_byte(data->slave_addr, OUT_MSB);
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to set OUT_MSB\n");
        return err;
    }

    // 결과 읽기
    err = my_i2c_read_bytes(data->slave_addr, rx, sizeof(rx));
    if (err != I2C_ERR_NONE) {
        pr_err("Failed to read temperature result\n");
        return err;
    }

    // 보정 계산
    ut = (rx[0] << 8) | rx[1];
    data->calib.B5 = bmp180_calc_b5(&data->calib, ut);
    *temp_out = (data->calib.B5 + 8) >> 4;

    return err;
}

static s32 bmp180_calc_b5(struct bmp180_calib *calib, u16 ut) {
    s32 x1, x2;
    x1 = ((ut - (s32)calib->AC6) * (s32)calib->AC5) >> 15;
    x2 = ((s32)calib->MC << 11) / (x1 + (s32)calib->MD);
    return x1 + x2;
}



static int bmp180_calc_pressure(struct bmp180_data *data, uint32_t up) {
    s32 b6, x1, x2, x3, b3, p;
    u32 b4, b7;
    struct bmp180_calib *c = &data->calib;

    b6 = data->calib.B5 - 4000;

    // Calculate B3
    x1 = (c->B2 * (b6 * b6 >> 12)) >> 11;
    x2 = (c->AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((s32)c->AC1) * 4 + x3) << data->oss) + 2) >> 2;

    // Calculate B4
    x1 = (c->AC3 * b6) >> 13;
    x2 = (c->B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((u32)c->AC4 * (u32)(x3 + 32768)) >> 15;

    b7 = ((u32)up - (u32)b3) * (50000 >> data->oss);
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    return p;
}


// ----- 모듈 init / exit -----
static int __init bmp180_raw_init(void)
{
    pr_debug("bmp180 sysfs init\n");

    i2c_error_t err = I2C_ERR_NONE;

    // class create
    bmp_class = class_create(BMP180_CLASS_NAME);
    if (IS_ERR(bmp_class)) {
        pr_err("Failed to create class\n");
        return PTR_ERR(bmp_class);
    }

    // device create
    bmp_dev = device_create(bmp_class, NULL, 0, NULL, BMP180_DEVICE_NAME);
    if (IS_ERR(bmp_dev)) {
        pr_err("Failed to create device\n");
        class_destroy(bmp_class);
        return PTR_ERR(bmp_dev);
    }

    // 동적할당
    bmp_data = kzalloc(sizeof(struct bmp180_data), GFP_KERNEL);
    if (!bmp_data) {
        pr_err("Failed to allocate bmp180_data\n");
        device_destroy(bmp_class, 0);
        class_destroy(bmp_class);
        return -ENOMEM;
    }

    bmp_data->slave_addr = DEVICE_ADDR;
    bmp_data->oss = 0;
    mutex_init(&bmp_data->lock);

    my_i2c_lock();
    err = my_i2c_register_device(bmp_data->slave_addr, I2C_DEV_GENERIC);
    if(err != I2C_ERR_NONE) {
        my_i2c_unlock();
        kfree(bmp_data);
        device_destroy(bmp_class, 0);
        class_destroy(bmp_class);
        pr_err("Failed to register device on i2c bus: %d\n", err);
        return -EIO;
    }

    err = my_i2c_ping(bmp_data->slave_addr);
    if(err != I2C_ERR_NONE) {
        my_i2c_unregister_device(DEVICE_ADDR);
        my_i2c_unlock();
        pr_err("No device found at address 0x%02X: %d\n", bmp_data->slave_addr, err);
        kfree(bmp_data);
        device_destroy(bmp_class, 0);
        class_destroy(bmp_class);
        return -EIO;
    }

    my_i2c_unlock();


    // group으로 한번에 sysfs에 등록
    return sysfs_create_group(&bmp_dev->kobj, &bmp_group);
}

static void __exit bmp180_raw_exit(void)
{
    pr_debug("bmp180 sysfs exit\n");

    my_i2c_lock();
    my_i2c_unregister_device(DEVICE_ADDR);
    my_i2c_unlock();

    if(bmp_data) {
        kfree(bmp_data);
    }

    sysfs_remove_group(&bmp_dev->kobj, &bmp_group);
    device_destroy(bmp_class, 0);
    class_destroy(bmp_class);
}

module_init(bmp180_raw_init);
module_exit(bmp180_raw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("BMP180 my_i2c sysfs attribute driver");
