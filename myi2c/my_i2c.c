// pr_debug, pr_info, pr_err 메시지 접두사 macro
#define pr_fmt(fmt) "[my_i2c] " fmt

#include "my_i2c.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>




/*
    i2c protocol을 위한 platform driver

*/


#define I2C_ADDR_WRITE(addr)  ((addr) << 0x01) // Write mode: LSB 0
#define I2C_ADDR_READ(addr)   (((addr) << 0x01) | 0x01)

#define DEVICE_NAME "my_i2c"
#define I2C_SDA_GPIO_NAME "myi2c-sda"
#define I2C_SCL_GPIO_NAME "myi2c-scl"

static const unsigned int i2c_delays_us[I2C_DELAY_COUNT] = {
    [I2C_DELAY_START_HOLD]    = 5,    // START hold time: 4.0us
    [I2C_DELAY_SCL_LOW_HOLD]  = 5,    // SCL LOW hold time: 4.7us → 여유 있게 5us
    [I2C_DELAY_SCL_HIGH_HOLD] = 5,    // SCL HIGH hold time: 4.0us
    [I2C_DELAY_STOP_SETUP]    = 5,    // STOP setup time: 4.0us
    [I2C_DELAY_SDA_HOLD]      = 6,    // SDA data hold time: 5.0us
    [I2C_DELAY_RESTART_COND]  = 5,    // Restart 조건 (Bus free time): 4.7us
    [I2C_DELAY_TEMP]          = 2,    // 임시 딜레이 : 2us
    [I2C_DELAY_ACK_TIMEOUT]   = 2     // ACK polling 간격: 2us
};


// my_i2c driver 데이터 구조체
struct my_i2c_data {
    struct gpio_desc *sda_desc;
    struct gpio_desc *scl_desc;
    i2c_state_t state; // 현재 I2C 상태
    int ready; // 드라이버 준비 상태
    struct mutex lock;
};

static struct my_i2c_data *g_my_i2c_data = NULL; // 전역 데이터 포인터


// 내부 함수 전방선언
static i2c_error_t my_i2c_master_ack(struct my_i2c_data *data, int ack);
static i2c_error_t my_i2c_start(struct my_i2c_data *data);
static i2c_error_t my_i2c_stop(struct my_i2c_data *data);
static i2c_error_t my_i2c_ack(struct my_i2c_data *data);
static i2c_error_t my_i2c_read(struct my_i2c_data *data, uint8_t *byte, int send_ack);
static i2c_error_t my_i2c_write(struct my_i2c_data *data, uint8_t byte);

void my_i2c_init_gpio(void) {
    pr_info("gpio init\n");

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return;
    }

    mutex_lock(&g_my_i2c_data->lock);

    // SDA: output + high
    gpiod_direction_output(g_my_i2c_data->sda_desc, HIGH);
    // SCL: output + high
    gpiod_direction_output(g_my_i2c_data->scl_desc, HIGH);

	udelay(i2c_delays_us[I2C_DELAY_TEMP]);

    mutex_unlock(&g_my_i2c_data->lock);
}


// my i2c slave pint 함수
i2c_error_t my_i2c_ping(uint8_t slave_addr) {
    pr_info("ping I2C address 0x%02X\n", slave_addr);

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    my_i2c_debug();

    i2c_error_t err;

    // START condition
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to start I2C bus\n");
        return err;
    }

    // WRITE address only, expect ACK
    err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(slave_addr));
    if (err == I2C_ERR_NONE) {
        pr_info("Found device at 0x%02X\n", slave_addr);
    } 
    else {
        pr_err("No ACK from device at 0x%02X (err=%d)\n", slave_addr, err);
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    my_i2c_stop(g_my_i2c_data);
    pr_info("I2C ping done.\n");
    return I2C_ERR_NONE;
}

// slave 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_bytes(uint8_t slave_addr, uint8_t *data, size_t len) {

    i2c_error_t err = I2C_ERR_NONE;

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }

    // 7bit address + Read(1)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_READ(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    for (size_t i = 0; i < len; ++i) {
        int ack = (i < len - 1) ? I2C_ACK : I2C_NACK;  // 마지막 바이트에는 NACK
        if ((err = my_i2c_read(g_my_i2c_data, &data[i], ack)) != I2C_ERR_NONE) {
            my_i2c_stop(g_my_i2c_data);
            return err;
        }
    }

    if ((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }

    return I2C_ERR_NONE;
}

// slave 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_bytes(uint8_t addr, const uint8_t *data, size_t len) {

    i2c_error_t err;

    if(!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }
    
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE)
        return err;

    // Write mode: LSB 0
    
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    for (size_t i = 0; i < len; ++i) {
        if ((err = my_i2c_write(g_my_i2c_data, data[i])) != I2C_ERR_NONE) {
            my_i2c_stop(g_my_i2c_data);
            return err;
        }
    }

    // Stop condition
    if((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }
    return I2C_ERR_NONE;
}

// 레지스터 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_reg(uint8_t slave_addr, uint8_t reg, const uint8_t *data, size_t len) {

    i2c_error_t err = I2C_ERR_NONE;

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    // START
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }

    // WRITE slave addr w
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // write register
    if ((err = my_i2c_write(g_my_i2c_data, reg)) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // WRITE data
    for (size_t i = 0; i < len; ++i) {
        if ((err = my_i2c_write(g_my_i2c_data, data[i])) != I2C_ERR_NONE) {
            my_i2c_stop(g_my_i2c_data);
            return err;
        }
    }

    return I2C_ERR_NONE;
}

// 레지스터 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_reg(uint8_t slave_addr, uint8_t reg, uint8_t *data, size_t len) {

    i2c_error_t err;

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    // write register addr
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }

    // write slave addr (write)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // write register
    if ((err = my_i2c_write(g_my_i2c_data, reg)) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // --- Repeated START + READ ---
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        return err;
    }

    // write slave addr (read)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_READ(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // read 
    for (size_t i = 0; i < len; ++i) {
        int ack = (i < len - 1) ? I2C_ACK : I2C_NACK;
        if ((err = my_i2c_read(g_my_i2c_data, &data[i], ack)) != I2C_ERR_NONE) {
            my_i2c_stop(g_my_i2c_data);
            return err;
        }
    }

    return I2C_ERR_NONE;
}

// 디버그용 함수
void my_i2c_debug(void) {

    if (!g_my_i2c_data) {
        return;
    }

    int sda1 = gpiod_get_value(g_my_i2c_data->sda_desc);
    int scl1 = gpiod_get_value(g_my_i2c_data->scl_desc);

    int sda_dir = gpiod_get_direction(g_my_i2c_data->sda_desc);  // 0: output, 1: input
    int scl_dir = gpiod_get_direction(g_my_i2c_data->scl_desc);

    pr_info("state %d SDA=%d (%s) SCL=%d (%s)\n",
        g_my_i2c_data->state,
        sda1,
        sda_dir == 0 ? "OUT" : "IN",
        scl1,
        scl_dir == 0 ? "OUT" : "IN");
}



// i2c start condition
static i2c_error_t my_i2c_start(struct my_i2c_data *data) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    my_i2c_init_gpio();

    mutex_lock(&data->lock);

    pr_info("my_i2c_start\n");
    data->state = I2C_STATE_START;

    my_i2c_debug();

    // SCL high일 때 sda low로 변환
	// SDA OUTPUT + LOW
	// start hold time : 4.0us 
    gpiod_direction_output(data->sda_desc, LOW);  // output 모드 + HIGH
	udelay(i2c_delays_us[I2C_DELAY_START_HOLD]);

    mutex_unlock(&data->lock);

    return I2C_ERR_NONE;
}
	
static i2c_error_t my_i2c_stop(struct my_i2c_data *data) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_STOP;

    // SCL이 HIGH일 때 SDA 라인이 LOW에서 HIGH로 전환

    // SDA를 LOW로 내리고 -> STOP 조건 위해 준비
    gpiod_direction_output(data->sda_desc, LOW);

    // stop setup time 4.0 us ( scl high hold time 4.0 us )
    // SCL을 HIGH로 올림 (SCL HIGH일 때 SDA LOW → HIGH 전이 발생해야 함)
    gpiod_set_value(data->scl_desc, HIGH);
    udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);


    // SDA HIGH stop condition
    // STOP condition (SCL HIGH 상태에서 SDA LOW → HIGH)
    gpiod_set_value(data->sda_desc, HIGH);

	// bus free time( sda high hold time 대체)
	// restart condition 4.7us
    udelay(i2c_delays_us[I2C_DELAY_RESTART_COND]);


    mutex_unlock(&data->lock);

    return I2C_ERR_NONE;
}


// slave의 receiver acknowledge
i2c_error_t my_i2c_ack(struct my_i2c_data *data) {

    int ack_received = 0;

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);
    data->state = I2C_STATE_ACK_CHECK;

    // // SDA input mode
    // gpiod_direction_input(data->sda_desc);


    int sda1 = gpiod_get_value(data->sda_desc);
    pr_info("ack pre clock : SDA=%d\n", sda1);


    // SCL LOW → clock falling edge
    gpiod_set_value(data->scl_desc, LOW);
    udelay(i2c_delays_us[I2C_DELAY_SCL_LOW_HOLD]);

    sda1 = gpiod_get_value(data->sda_desc);
    pr_info("ack low clock : SDA=%d\n", sda1);


    // scl low to high
    // scl high hold time 4.0us
    gpiod_set_value(data->scl_desc, HIGH);
    udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);

    sda1 = gpiod_get_value(data->sda_desc);
    pr_info("ack high clock : SDA=%d\n", sda1);


    // ACK 확인 루프
    for (int i = 0; i < I2C_ACK_TIMEOUT_US; i+=i2c_delays_us[I2C_DELAY_ACK_TIMEOUT]) {

        int sda = gpiod_get_value(data->sda_desc);
        //pr_info("ACK Check %d: SDA=%d\n", i, sda);
        if (sda == I2C_ACK) {
            ack_received = 1;
            break;
        }

        udelay(i2c_delays_us[I2C_DELAY_ACK_TIMEOUT]);
    }


    mutex_unlock(&data->lock);

    // ACK 수신 실패
    if(!ack_received) {
        pr_err("ACK not received\n");
        return I2C_ERR_NO_ACK; 
    }

    return I2C_ERR_NONE;
}

// master의 receiver acknowledge
i2c_error_t my_i2c_master_ack(struct my_i2c_data *data, int ack) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_MASTER_ACK;

    // scl은 HIGH 상태여야 함

    // SCL LOW 상태에서 SDA 출력 설정
    // scl low hold time 4.7us
    gpiod_set_value(data->scl_desc, LOW);
    udelay(i2c_delays_us[I2C_DELAY_SCL_LOW_HOLD]);

    // SDA를 출력 모드 + ack 값 설정 (0: ACK, 1: NACK)
    gpiod_direction_output(data->sda_desc, ack);

    // scl low to high
    // scl high hold time 4.0us
    gpiod_set_value(data->scl_desc, HIGH);
    udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);

    mutex_unlock(&data->lock);

    return I2C_ERR_NONE;
}


static i2c_error_t my_i2c_write(struct my_i2c_data *data, uint8_t byte) {

    i2c_error_t ret = I2C_ERR_NONE;

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_SEND_DATA;

    pr_info("my_i2c_write: byte=0x%02X\n", byte);
	// write byte

    // 여기 들어올 때 scl은 HIGH 상태여야 함
	
    // write 루프 전에 출력 모드로 전환
    gpiod_direction_output(data->sda_desc, LOW);

	// msb to lsb 
    for (int i = 7; i >= 0; --i) {

        // scl low
        // scl low hold time 4.7us
        gpiod_set_value(data->scl_desc, LOW);
        udelay(i2c_delays_us[I2C_DELAY_SCL_LOW_HOLD]);

        // SDA 출력 설정 + bit write
        // sda hold time 5.0us
        gpiod_set_value(data->sda_desc, (byte >> i) & 0x01);
        udelay(i2c_delays_us[I2C_DELAY_SDA_HOLD]);
        my_i2c_debug();

        
        // scl high setup time 250ns
        // 이 time은 sda hold time에 종속됨
        // udelay(0.25); // 250ns

        // scl low to high
        // scl high hold time 4.0us
        gpiod_set_value(data->scl_desc, HIGH);
        udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);
    }


    // 확인
    pr_info("after write\n");
    my_i2c_debug();


    // ack
    // scl을 먼저 low로 만듦 ( 9번째 클럭 펄스 생성 )
    // SCL LOW → clock falling edge
    gpiod_set_value(data->scl_desc, LOW);
    udelay(i2c_delays_us[I2C_DELAY_SCL_LOW_HOLD]);

    pr_info("after low clock\n");
    my_i2c_debug();

    // input 모드로 전환 ( SDA 라인 해제 )
    gpiod_direction_input(data->sda_desc);
    udelay(i2c_delays_us[I2C_DELAY_TEMP]);

    pr_info("after inputmode clock\n");
    my_i2c_debug();

    // scl low to high
    // scl high hold time 4.0us 
    // timeout이 있으므로 즉시 sda를 sampling 하도록 hole time 없음
    gpiod_set_value(data->scl_desc, HIGH);
    //udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);

    pr_info("after high clock clock\n");
    my_i2c_debug();

    // slave가 성공적으로 수신하면 SDA를 LOW로 당김 ( SCL 이 HIGH인 기간동인 LOW 상태 유지)

    // ACK 확인 루프
    int ack_received = 0;
    for (int i = 0; i < I2C_ACK_TIMEOUT_US; i+=i2c_delays_us[I2C_DELAY_ACK_TIMEOUT]) {

        int sda = gpiod_get_value(data->sda_desc);
        //pr_info("ACK Check %d: SDA=%d\n", i, sda);
        if (sda == I2C_ACK) {
            ack_received = 1;
            break;
        }

        udelay(i2c_delays_us[I2C_DELAY_ACK_TIMEOUT]);
    }

    // SCL을 다시 LOW로 
    gpiod_set_value(data->scl_desc, LOW);
    udelay(i2c_delays_us[I2C_DELAY_SCL_LOW_HOLD]);


    mutex_unlock(&data->lock);

    if(ack_received) {
        pr_info("ACK received\n");
    } else {
        pr_err("ACK not received\n");
        return I2C_ERR_NO_ACK;
    }

    // // ACK check
    // if ((ret = my_i2c_ack(data)) != I2C_ERR_NONE) {
    //     return ret;
    // }

    return I2C_ERR_NONE;
}

static i2c_error_t my_i2c_read(struct my_i2c_data *data, uint8_t *byte, int send_ack) {

    i2c_error_t ret = I2C_ERR_NONE;
    int read_data = 0;

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    // SDA input mode
    gpiod_direction_input(data->sda_desc);

    for (int i = 0; i < 8; ++i) {
        // SCL low hold time 4.7us
        gpiod_set_value(data->scl_desc, LOW);
        udelay(i2c_delays_us[I2C_DELAY_SDA_HOLD]);

        // SCL high hold time 4.0us
        gpiod_set_value(data->scl_desc, HIGH);
        udelay(i2c_delays_us[I2C_DELAY_SCL_HIGH_HOLD]);

        // 3. SDA 상태 읽기
        read_data <<= 0x01;
        if (gpiod_get_value(data->sda_desc)) {
            read_data |= 0x01;  // SDA가 HIGH면 1
        }
    }

    mutex_unlock(&data->lock);

    // Master ACK/NACK
    if ((ret = my_i2c_master_ack(data, send_ack)) != I2C_ERR_NONE) {
        return ret;
    }

    *byte = read_data;

    return ret;
}

/* --- 플랫폼 드라이버 등록 --- */
static int my_i2c_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct my_i2c_data *data;
    
    pr_info("platform driver probed started\n");
    
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        return -ENOMEM;
    }
        
    // device tree gpio와 매칭
    // devm은 리소스 자동 관리

    // sda : output + high
    data->sda_desc = devm_gpiod_get(dev, I2C_SDA_GPIO_NAME, GPIOD_OUT_HIGH);
    if (IS_ERR(data->sda_desc)) {
        pr_err("Failed to get sda_desc: %ld\n", PTR_ERR(data->sda_desc));
        return PTR_ERR(data->sda_desc);
    }

    // scl : output + high ( 초기 상태)
    data->scl_desc = devm_gpiod_get(dev, I2C_SCL_GPIO_NAME, GPIOD_OUT_HIGH);
    if (IS_ERR(data->scl_desc)) {
        pr_err("Failed to get scl_desc: %ld\n", PTR_ERR(data->scl_desc));
        return PTR_ERR(data->scl_desc);
    }


    // 초기 상태 설정
    data->state = I2C_STATE_IDLE;


    // mutex 초기화
    mutex_init(&data->lock);

    // 플랫폼 디바이스와 this driver간 data를 공유하도록 set
    platform_set_drvdata(pdev, data);
    g_my_i2c_data = data;

    my_i2c_debug();

    // rotary ready ok
    data->ready = 1;
    pr_info("platform driver probed successfully\n");
    
    return 0;
}

static void my_i2c_remove(struct platform_device *pdev) {
    struct my_i2c_data *data = platform_get_drvdata(pdev);
    
    pr_info("my_i2c_remove\n");
    
    if (data) {

        data->ready = 0;

        g_my_i2c_data = NULL;

    }
}



/* Device Tree 매칭 테이블 */
static const struct of_device_id my_i2c_of_match[] = {
    { .compatible = "chan,my-i2c", }, // DTS의 compatible 문자열과 일치해야함
    { }
};
MODULE_DEVICE_TABLE(of, my_i2c_of_match);

// platfomr driver 구조체
static struct platform_driver my_i2c_driver = {
    .probe = my_i2c_probe,
    .remove = my_i2c_remove,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = my_i2c_of_match,
    },
};

static int __init my_i2c_init(void) {
    pr_info("driver init\n");
    return platform_driver_register(&my_i2c_driver);
}

static void __exit my_i2c_exit(void) {
    pr_info("driver exit\n");
    platform_driver_unregister(&my_i2c_driver);
}

module_init(my_i2c_init);
module_exit(my_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY I2C Platform Device Driver");

// Exported functions
EXPORT_SYMBOL(my_i2c_init_gpio);
EXPORT_SYMBOL(my_i2c_debug);
EXPORT_SYMBOL(my_i2c_ping);
EXPORT_SYMBOL(my_i2c_read_bytes);
EXPORT_SYMBOL(my_i2c_write_bytes);
EXPORT_SYMBOL(my_i2c_write_reg);
EXPORT_SYMBOL(my_i2c_read_reg);