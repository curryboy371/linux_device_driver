#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/device.h>


/*
 * BMP180 Linux I2C 드라이버 (sysfs + 모듈 파라미터로 i2c 어댑터 지정)
 *
 * - i2c_driver 구조체를 사용한 Linux I2C 클라이언트 드라이버
 * - 사용자 공간과 sysfs를 통해 직접 값 읽기/쓰기 가능:
 *     /sys/bus/i2c/devices/1-0077/temperature
 *     /sys/bus/i2c/devices/1-0077/pressure
 *   cat, echo로 직접 측정값 확인 및 설정 가능.
 *
 * - 모듈 파라미터로 i2c 어댑터(bus) 번호 지정 가능:
 *
 * - module_init에서 i2c_get_adapter + i2c_new_client_device 호출로
 *   별도의 device tree 없이 I2C 장치를 직접 등록.
 *
 * - 이후 i2c-core가 probe() 호출해 센서 초기화 및 보정값 로드, sysfs 파일 생성.
 *
 * - 사용 예시:
 *     cat /sys/bus/i2c/devices/1-0077/temperature
 *     echo 3 > /sys/bus/i2c/devices/1-0077/pressure   # oss 값 설정
 *     cat /sys/bus/i2c/devices/1-0077/pressure
 *
 */

// slave address
#define BMP180_NAME     "BMP180"
#define BMP180_ADDR     0x77

// busnum을 파라미터화
// insmod bmp180.ko busnum=2
static int busnum = 1;
module_param(busnum, int, 0644);
MODULE_PARM_DESC(busnum, "I2C bus number to attach BMP180");


//  보정 테이블 레지스터 주소 0xAA부터
#define CALIB_DATA_START    0xAA
#define CALIB_DATA_END      0xBF
#define CALIB_DATA_LENGTH   (CALIB_DATA_END - CALIB_DATA_START + 1)

// 측정 데이터 저장 레지스터
#define OUT_MSB     0xF6    // ro
#define OUT_LSB     0xF7    // ro
#define OUT_XLSB    0xF8    // ro

// 명령 레지스터
#define CTRL_MEAS   0xF4    // rw 측정 시작 명령어
#define SOFT_RESET  0xE0    // rw 센서 초기화
#define ID          0xD0    // ro 장치 ID 확인용

#define SOFT_RESET_VALUE 0xB6   // 초기화 값

// 측정 타입 레지스터
#define TEMP    0x2E        // 온도 측정 명령
#define PRESSURE 0x34       // 압력 측정 oss 0

#define BMP_MSB     0
#define BMP_LSB     1
#define BMP_XLSB    2

// calib 구조체
struct bmp180_calib {
	s16 AC1;
	s16 AC2;
	s16 AC3;
	u16 AC4;
	u16 AC5;
	u16 AC6;
	s16 B1;
	s16 B2;
	s16 MB;
	s16 MC;
	s16 MD;

	s16 B5; // 레지스터 read 가 아닌 중간 계산값

};

// bmp180 data 구조체
struct bmp180_data {
    struct i2c_client *client;
    struct bmp180_calib calib;
    u8 oss;
};

static struct i2c_client *bmp180_client;
static const int OSS_WAIT_TIME_MS[4] = { 5, 8, 14, 26 };

// 전방선언
static int bmp180_read_temperature(struct bmp180_data* data, s32* out_temp);
static int bmp180_read_pressure(struct bmp180_data* data, s32* out_pressure);
static int bmp180_read_calib(struct i2c_client* client, struct bmp180_calib* calib);
static s32 bmp180_calc_temperature(struct bmp180_data* data, s32 ut);
static s32 bmp180_calc_pressure(struct bmp180_data* data, s32 up);
static ssize_t temperature_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t pressure_show(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t pressure_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);

// linux/device.h
//static DEVICE_ATTR(temperature);
//static DEVICE_ATTR(pressure);

// attribute 구조체 정의
// show, store 함수 등록
static struct device_attribute dev_attr_temperature = {
    .attr = {
        .name = "temperature",
        .mode  = 0666,
    },
    .show = temperature_show,
};

static struct device_attribute dev_attr_pressure = {
    .attr = {
        .name = "pressure",
        .mode  = 0666,
    },
    .show = pressure_show,
    .store = pressure_store,
};


// attribute array 생성
static struct attribute *bmp180_attrs[] = {
    &dev_attr_temperature.attr,
    &dev_attr_pressure.attr,
    NULL,
};

// grroup화
static const struct attribute_group bmp180_group = {
    .attrs = bmp180_attrs,
};

// temp show 함수
static ssize_t temperature_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    struct bmp180_data *data = dev_get_drvdata(dev);
    s32 temperature = 0;
    int ret;

    ret = bmp180_read_temperature(data, &temperature);
    if (ret < 0)
        return ret;

    return sprintf(buf, "%d.%d\n", temperature / 10, temperature % 10);
}

// pressure show 함수
static ssize_t pressure_show(struct device* dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct bmp180_data *data = dev_get_drvdata(dev);
    s32 pressure = 0;
    ret = bmp180_read_pressure(data, &pressure);
    if (ret < 0)
        return ret;

    return sprintf(buf, "%d.%d\n", pressure / 100, pressure % 100);
}

// oss값을 입력받는 함수 ( pressure에만 oss가 사용되므로 presure의 store로 사용)
static ssize_t pressure_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    // oss를 설정하는 함수

    int ret, value;

    struct bmp180_data *data = dev_get_drvdata(dev);

    // 정수형 변환
    ret = kstrtoint(buf, 10, &value);
    if (ret < 0)
        return ret;

    // 유효 범위
    if (value < 0 || value > 3)
        return -EINVAL;

    data->oss = value;

    return count;
}


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
static int bmp180_read_calib(struct i2c_client* client, struct bmp180_calib* calib)
{
	int ret;

	u8 data[CALIB_DATA_LENGTH];

    // 보정 테이블 register read 
    ret = i2c_smbus_read_i2c_block_data(client, CALIB_DATA_START, CALIB_DATA_LENGTH, data);
    if (ret < 0) {
        pr_err("Failed to read calib data\n");
        return ret;
    }

    int16_t* table[] = {
        &calib->AC1, &calib->AC2, &calib->AC3,
        (int16_t*)&calib->AC4, (int16_t*)&calib->AC5, (int16_t*)&calib->AC6,
        &calib->B1, &calib->B2,
        &calib->MB, &calib->MC, &calib->MD
    };

    // register data calib에 저장
    for (int i = 0; i < CALIB_DATA_LENGTH / 2; i++) {
        *table[i] = (int16_t)(data[2 * i] << 8 | data[2 * i + 1]);
    }

    pr_info("Calibration table loaded\n");
	return 0;
}

// 온도 읽기
static int bmp180_read_temperature(struct bmp180_data* data, s32* out_temp)
{
	int ret = 0;
	struct i2c_client* client = data->client;

    // 측정 명령
	ret = i2c_smbus_write_byte_data(client, CTRL_MEAS, TEMP); // 측정, 온도
	if (ret < 0)
		return ret;

	msleep(OSS_WAIT_TIME_MS[0]);

    *out_temp = 0;

    // 측정 결과 MSB 부터 read
    u8 rx_data[2];
    ret = i2c_smbus_read_i2c_block_data(client, OUT_MSB, 2, rx_data);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read OUT_MSB\n");
        return ret;
    }

    // 측정 값 보정 연산
    s32 u_temp = (rx_data[BMP_MSB] << 8) | rx_data[BMP_LSB];
    *out_temp = bmp180_calc_temperature(data, u_temp);

	return 0;
}

// 압력 읽기
static int bmp180_read_pressure(struct bmp180_data* data, s32* out_pressure)
{
    int ret;
    struct i2c_client *client = data->client;

    // 측정 명령
    u8 ctrl_value = PRESSURE | (data->oss << 6); // 측정, 압력
    ret = i2c_smbus_write_byte_data(client, CTRL_MEAS, ctrl_value);
    if (ret < 0)
        return ret;

    // wait
    msleep(OSS_WAIT_TIME_MS[data->oss]);

    *out_pressure = 0;

    // 측정 read, MSB부터 read
    u8 rx_data[3];
    ret = i2c_smbus_read_i2c_block_data(client, OUT_MSB, 3, rx_data);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read pressure data: %d\n", ret);
        return ret;
    }

    // uncompensated pressure 계산
    s32 u_pressure = ((rx_data[BMP_MSB] << 16) | (rx_data[BMP_LSB] << 8) | rx_data[BMP_XLSB] >> (8 - data->oss));

    *out_pressure = bmp180_calc_pressure(data, u_pressure);

	return 0;
}

// 디바이스 등록될 때 호출
static int bmp180_probe(struct i2c_client* client)
{
	int ret;
	struct bmp180_data* data = NULL;

    pr_info("start bmp180 probed\n");

    // 드라이버 전용 구조체 할당
    data = devm_kzalloc(&client->dev, sizeof(struct bmp180_data), GFP_KERNEL);
    if (!data)
        return -1;

    data->client = client;
	data->oss = 0;

    dev_set_drvdata(&client->dev, data);

    // sysfs attribute 등록
    // /sys/bus/i2c/devices/1-0077/.... 하위 등록 value 생성
    ret = sysfs_create_group(&client->dev.kobj, &bmp180_group);         // group으로 한번에
    //ret = device_create_file(&client->dev, &dev_attr_temperature);
    //ret = device_create_file(&client->dev, &dev_attr_pressure);

    if (ret)
        return ret;


    // 센서 초기화
    ret = i2c_smbus_write_byte_data(client, SOFT_RESET, SOFT_RESET_VALUE);
    if (ret < 0) {
        return ret;
    }

    msleep(10);    

    // 보정값 읽기 (여기서 data->calib 초기화 필요)
	ret = bmp180_read_calib(client, &data->calib);
	if (ret < 0)
		return ret;    


    pr_info("finished BMP180 probed\n");
    return 0;
}

// 장치가 제거되거나 unload 되는 경우 호출
static void bmp180_remove(struct i2c_client* client) {

    sysfs_remove_group(&client->dev.kobj, &bmp180_group);
    pr_info("bmp180_remove\n");
}


// id table 매칭 사용
// struct i2c_board_info bmp180_info = {
// 	.type = "bmp180",
// 	.addr = 0x77,
// };
static const struct i2c_device_id bmp180_id[] = {
	{BMP180_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

static struct i2c_driver bmp180_driver = {

    .driver = {
		.name	= BMP180_NAME,
		//.of_match_table = bmp180_of_i2c_match,
	},
	.probe		= bmp180_probe,
    .remove     = bmp180_remove,
	.id_table	= bmp180_id,
};


// 모듈 insert 되었을 때 호출
static int __init bmp180_init(void)
{

    int ret;
    struct i2c_adapter* adapter = NULL;
    struct i2c_board_info info = { I2C_BOARD_INFO(BMP180_NAME, BMP180_ADDR)};

    pr_info("start bmp180_init\n");

    // i2c 디바이스 등록 ( module_i2c_driver )
    ret = i2c_add_driver(&bmp180_driver);
    if (ret)
        return ret;

   
    // bus 어뎁터 가져오기
    adapter = i2c_get_adapter(busnum);  
    if (!adapter) {
        i2c_del_driver(&bmp180_driver);
        return -ENODEV;
    }

    // 디바이스 등록 ( 장치 이름, 주소)
    // 유저의 new_device 등록을 수행
    // Device Tree에서 등록하여 부팅시 바인딩하는게 좋다고 함
    bmp180_client = i2c_new_client_device(adapter, &info);

    // bus 어뎁터 반환
    i2c_put_adapter(adapter);

    if (!bmp180_client) {
        i2c_del_driver(&bmp180_driver);
        return -ENODEV;
    }

    pr_info("finished bmp180_init\n");

    return 0;
}

static void __exit bmp180_exit(void)
{
    pr_info("start bmp180_exit\n");

    if (bmp180_client)
        i2c_unregister_device(bmp180_client);

    // i2c 디바이스 해제 ( module_i2c_driver )
    i2c_del_driver(&bmp180_driver);

    pr_info("finished bmp180_exit\n");
}

//module_i2c_driver(bmp180_driver);
module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("BMP180 sysfs attribute driver");
