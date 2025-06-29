#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/device.h>

/*
 * BH1750 Linux I2C Device Driver (sysfs + 모듈 파라미터로 i2c 어댑터 지정)
 *
 * - i2c_driver 구조체를 사용한 표준 Linux I2C 클라이언트 드라이버
 * - 사용자 공간과 sysfs를 통해 직접 값 읽기/쓰기 가능:
 *     /sys/bus/i2c/devices/1-0023/value
 *     /sys/bus/i2c/devices/1-0023/mode
 *   cat, echo 로 직접 측정값 확인 및 센서 모드 설정 가능.
 *
 * - 모듈 파라미터로 i2c 어댑터(bus) 번호 지정 가능:
 *     insmod bh1750.ko busnum=2
 *
 * - module_init 에서 i2c_add_driver 로 등록 후
 *   i2c_get_adapter + i2c_new_client_device 로 장치 수동 등록.
 *   이후 i2c-core 가 자동으로 probe() 호출.
 */

#define BH1750_I2C_ADDR   0x23
#define BH1750_NAME       "bh1750"

static int busnum = 1;
module_param(busnum, int, 0644);
MODULE_PARM_DESC(busnum, "I2C bus number to attach BH1750");

static struct i2c_client *bh1750_client;

#define BH1750_CMD_POWER_DOWN    0x00
#define BH1750_CMD_POWER_ON      0x01
#define BH1750_CMD_RESET         0x07
#define BH1750_CMD_CONT_L        0x13
#define BH1750_CMD_ONETIME_L     0x23

struct bh1750_data {
    struct i2c_client *client;
    unsigned int current_mode;
};

// sysfs value
static ssize_t bh1750_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    u8 buffer[2];
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, 0, 2, buffer);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read measurement data\n");
        return ret;
    }
    if (ret != 2)
        return -EIO;

    u16 data = (buffer[0] << 8) | buffer[1];
    return sprintf(buf, "%u\n", data);
}
static DEVICE_ATTR(value, 0444, bh1750_show_value, NULL);

// sysfs mode
static ssize_t bh1750_store_mode(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bh1750_data *data = dev_get_drvdata(dev);
    unsigned long mode;
    int ret;

    ret = kstrtoul(buf, 10, &mode);
    if (ret)
        return ret;

    if (mode == 0)
        ret = i2c_smbus_write_byte(client, BH1750_CMD_POWER_DOWN);
    else if (mode == 1)
        ret = i2c_smbus_write_byte(client, BH1750_CMD_CONT_L);
    else
        return -EINVAL;

    if (ret < 0)
        return ret;

    data->current_mode = mode;
    return count;
}
static ssize_t bh1750_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bh1750_data *data = dev_get_drvdata(dev);
    return sprintf(buf, "%u\n", data->current_mode);
}
static DEVICE_ATTR(mode, 0664, bh1750_show_mode, bh1750_store_mode);

// i2c probe
static int bh1750_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct bh1750_data *data;

    dev_info(&client->dev, "start bh1750 probe at 0x%02x\n", client->addr);

    data = devm_kzalloc(&client->dev, sizeof(struct bh1750_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    data->current_mode = 0;
    dev_set_drvdata(&client->dev, data);

    i2c_smbus_write_byte(client, BH1750_CMD_POWER_ON);

    ret = device_create_file(&client->dev, &dev_attr_value);
    if (ret)
        return ret;

    ret = device_create_file(&client->dev, &dev_attr_mode);
    if (ret)
        return ret;

    dev_info(&client->dev, "finish bh1750 probe at 0x%02x\n", client->addr);
    return 0;
}

static void bh1750_remove(struct i2c_client *client)
{
    i2c_smbus_write_byte(client, BH1750_CMD_POWER_DOWN);
    device_remove_file(&client->dev, &dev_attr_value);
    device_remove_file(&client->dev, &dev_attr_mode);
    dev_info(&client->dev, "remove bh1750\n");
}

// id table
static const struct i2c_device_id bh1750_ids[] = {
    { BH1750_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bh1750_ids);

static struct i2c_driver bh1750_driver = {
    .driver = {
        .name = BH1750_NAME,
    },
    .probe = bh1750_probe,
    .remove = bh1750_remove,
    .id_table = bh1750_ids,
};

static int __init bh1750_init(void)
{
    int ret;
    struct i2c_adapter *adapter;
    struct i2c_board_info info = {
        I2C_BOARD_INFO(BH1750_NAME, BH1750_I2C_ADDR),
    };

    pr_info("bh1750: module init, registering driver\n");
    ret = i2c_add_driver(&bh1750_driver);
    if (ret)
        return ret;

    // busnum(default /dev/i2c-1) 어댑터를 가져옴
    adapter = i2c_get_adapter(busnum);
    if (!adapter) {
        i2c_del_driver(&bh1750_driver);
        return -ENODEV;
    }

    // 지정된 어댑터(bus)에 I2C 장치를 직접 등록
    // device tree 없이도 I2C 장치를 수동 등록
    // /sys/bus/i2c/devices/1-0023/.. 경로 생성됨
    bh1750_client = i2c_new_client_device(adapter, &info);
    i2c_put_adapter(adapter);

    if (IS_ERR(bh1750_client)) {
        i2c_del_driver(&bh1750_driver);
        return PTR_ERR(bh1750_client);
    }

    pr_info("bh1750: registered on i2c-%d at 0x%02x\n", busnum, BH1750_I2C_ADDR);
    return 0;
}

static void __exit bh1750_exit(void)
{
    if (bh1750_client)
        i2c_unregister_device(bh1750_client);

    i2c_del_driver(&bh1750_driver);
    pr_info("bh1750: module exit\n");
}

module_init(bh1750_init);
module_exit(bh1750_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("BH1750 sysfs I2C Driver");
