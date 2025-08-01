// pr_debug, pr_info, pr_err 메시지 접두사 macro
#define pr_fmt(fmt) "[my_i2c] " fmt


// pr_debuf 출력
#define DEBUG

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

#define MAX_I2C_SLAVES 12

#define RETRY_COUNT 3

// 등록된 슬레이브 주소 저장
static uint8_t registered_addrs[MAX_I2C_SLAVES] = { 0 };
static int slave_count = 0;

static const unsigned int i2c_delays_us[I2C_DEV_COUNT][I2C_DELAY_COUNT] = {

 
    // I2C 문서에 따른 Standard 셋업 홀드타임
    [I2C_DEV_GENERIC] = {
        [I2C_DELAY_START_SETUP]   = 1,  // 일반적으로 최소 0.6us, 1us 여유
        [I2C_DELAY_START_HOLD]    = 5,  // 4.0us 이상
        [I2C_DELAY_STOP_SETUP]    = 5,  // 4.0us 이상
        [I2C_DELAY_STOP_HOLD]     = 1,  // 일반적으로 0us 이상 → 1us로 설정
        [I2C_DELAY_SCL_LOW_HOLD]  = 6,  // 4.7us 이상
        [I2C_DELAY_SCL_HIGH_HOLD] = 5,  // 4.0us 이상
        [I2C_DELAY_SDA_SETUP]     = 1,  // 최소 100ns → 1us로 설정
        [I2C_DELAY_SDA_HOLD]      = 3,  // 0-3.45
        [I2C_DELAY_RESTART_COND]  = 5,  // Bus free time: 4.7us 이상
        [I2C_DELAY_ACK_TIMEOUT]   = 4,  // ACK polling 대기
        [I2C_DELAY_TEMP]          = 2   // 임시 지연
    },
    // LCD 문서에 따른 setup hold time
    [I2C_DEV_LCD1602] = {
        [I2C_DELAY_START_SETUP]   = 1,  // LCD도 START 조건은 기본 준수
        [I2C_DELAY_START_HOLD]    = 5,
        [I2C_DELAY_STOP_SETUP]    = 5,
        [I2C_DELAY_STOP_HOLD]     = 1,
        [I2C_DELAY_SCL_LOW_HOLD]  = 5,
        [I2C_DELAY_SCL_HIGH_HOLD] = 5,
        [I2C_DELAY_SDA_SETUP]     = 1,
        [I2C_DELAY_SDA_HOLD]      = 1,
        [I2C_DELAY_RESTART_COND]  = 5,
        [I2C_DELAY_ACK_TIMEOUT]   = 2,
        [I2C_DELAY_TEMP]          = 2
    }

};



// my_i2c driver 데이터 구조체
struct my_i2c_data {
    struct gpio_desc *sda_desc;
    struct gpio_desc *scl_desc;
    i2c_device_type_t type; // 등록된 드라이버의 type
    i2c_state_t state; // 현재 I2C 상태
    int ready; // 드라이버 준비 상태
    struct mutex lock;
};

static struct my_i2c_data *g_my_i2c_data = NULL; // 전역 데이터 포인터


// 내부 함수 전방선언
static i2c_error_t my_i2c_master_ack(struct my_i2c_data *data, int ack);
static i2c_error_t my_i2c_start(struct my_i2c_data *data);
static i2c_error_t my_i2c_restart(struct my_i2c_data *data);
static i2c_error_t my_i2c_stop(struct my_i2c_data *data);
static i2c_error_t my_i2c_ack(struct my_i2c_data *data);
static i2c_error_t my_i2c_read(struct my_i2c_data *data, uint8_t *byte, int send_ack);
static i2c_error_t my_i2c_write(struct my_i2c_data *data, uint8_t byte);


// slave register & unregister
i2c_error_t my_i2c_register_device(uint8_t slave_addr, i2c_device_type_t type) {
    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready, cannot register device.\n");
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&g_my_i2c_data->lock);

    g_my_i2c_data->type = type;

    pr_info("registered slave address : ");
    for (int i = 0; i < MAX_I2C_SLAVES; ++i) {
        if (registered_addrs[i] != 0x00)
            pr_cont("0x%02X ", registered_addrs[i]);
    }
    pr_cont("\n");


    // 이미 등록된 주소인지 확인
    for (int i = 0; i < MAX_I2C_SLAVES; ++i) {
        if (registered_addrs[i] == slave_addr) {
            mutex_unlock(&g_my_i2c_data->lock);
            pr_err("Device 0x%02X already registered.\n", slave_addr);
            return I2C_ERR_DUPLICATE;
        }
    }

    // 비어있는 위치에 등록
    for (int i = 0; i < MAX_I2C_SLAVES; ++i) {
        if (registered_addrs[i] == 0x00) {
            registered_addrs[i] = slave_addr;
            slave_count++;
            mutex_unlock(&g_my_i2c_data->lock);
            pr_info("Registered I2C device at address 0x%02X\n", slave_addr);
            return I2C_ERR_NONE;
        }
    }

    mutex_unlock(&g_my_i2c_data->lock);
    pr_err("No space to register more I2C devices slave num:%d\n", slave_count);
    return I2C_ERR_OVERFLOW;
}

// Unregister an I2C slave device
i2c_error_t my_i2c_unregister_device(uint8_t slave_addr) {

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready, cannot unregister device.\n");
        return I2C_ERR_NOT_READY;
    }

    
    mutex_lock(&g_my_i2c_data->lock);

    for (int i = 0; i < MAX_I2C_SLAVES; ++i) {
        if (registered_addrs[i] == slave_addr) {
            registered_addrs[i] = 0x00;
            slave_count--;
            pr_info("Unregistered I2C device at address 0x%02X\n", slave_addr);
            mutex_unlock(&g_my_i2c_data->lock);
            return I2C_ERR_NONE;
        }
    }

    mutex_unlock(&g_my_i2c_data->lock);

    pr_warn("Device 0x%02X was not registered.\n", slave_addr);
    return I2C_ERR_INVALID_ARG;
}

// my i2c slave pint 함수
i2c_error_t my_i2c_ping(uint8_t slave_addr) {
    pr_info("ping I2C address 0x%02X\n", slave_addr);

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    // 이미 등록된 주소인지 확인
    int find_addr = 0;
    for (int i = 0; i < MAX_I2C_SLAVES; ++i) {
        if (registered_addrs[i] == slave_addr) {
            find_addr =1;
        }
    }

    if(!find_addr) {
        pr_err("unregister addr\n");
        return I2C_INAVLID_ADDR;
    }

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
        my_i2c_stop(g_my_i2c_data);
    } 
    else {
        pr_err("No ACK from device at 0x%02X (err=%d)\n", slave_addr, err);
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    my_i2c_debug();
    
    pr_info("I2C ping done.\n");
    return I2C_ERR_NONE;
}


// slave 주소에서 바이트 읽기
i2c_error_t my_i2c_read_byte(uint8_t slave_addr, uint8_t *out_data) {
    return my_i2c_read_bytes(slave_addr, out_data, 1);
}

// slave 주소에 바이트 쓰기
i2c_error_t my_i2c_write_byte(uint8_t addr, const uint8_t data) {
    return my_i2c_write_bytes(addr, &data, 1, 0);
}

// 레지스터 주소에 바이트 쓰기
i2c_error_t my_i2c_write_reg_byte(uint8_t slave_addr, uint8_t reg, const uint8_t data) {
    return my_i2c_write_reg_bytes(slave_addr, reg, &data, 1, 0);
}

// 레지스터 주소에서 바이트 읽기
i2c_error_t my_i2c_read_reg_byte(uint8_t slave_addr, uint8_t reg, uint8_t* out_data) {
    return my_i2c_read_reg_bytes(slave_addr, reg, out_data, 1);
}

// slave 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_bytes(uint8_t slave_addr, uint8_t *out_data, size_t len) {

    i2c_error_t err = I2C_ERR_NONE;

    pr_debug("my_i2c_read_bytes %zu byte(s) from 0x%02X\n", len, slave_addr);

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to send START condition\n");
        return err;
    }

    // 7bit address + Read(1)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_READ(slave_addr))) != I2C_ERR_NONE) {
        pr_err("Failed to write slave addr\n");
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    for (size_t i = 0; i < len; ++i) {
        int ack = (i < len - 1) ? I2C_ACK : I2C_NACK;  // 마지막 바이트에는 NACK
        if ((err = my_i2c_read(g_my_i2c_data, &out_data[i], ack)) != I2C_ERR_NONE) {
            pr_err("Failed to read\n");
            my_i2c_stop(g_my_i2c_data);
            return err;
        }
    }

    if ((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to stop\n");
        return err;
    }

    return I2C_ERR_NONE;
}

// slave 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_bytes(uint8_t addr, const uint8_t *data, size_t len, int delay_us) {

    i2c_error_t err = I2C_ERR_NONE;

    if(!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    for (int retry = 0; retry < RETRY_COUNT; ++retry) {

        pr_debug("my_i2c_write_bytes retry(%d) %zu byte(s) to 0x%02X\n", retry, len, addr);

        if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
            pr_err("START failed\n");
            continue;
        }

        if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(addr))) != I2C_ERR_NONE) {
            pr_err("Write address (0x%02X) NACK\n", I2C_ADDR_WRITE(addr));
            my_i2c_stop(g_my_i2c_data);
            continue;
        }

        // Write address (WRITE mode)

        uint8_t success = 1;
        for (size_t i = 0; i < len; ++i) {

            pr_debug("Writing data[%zu] = 0x%02X\n", i, data[i]);
            if ((err = my_i2c_write(g_my_i2c_data, data[i])) != I2C_ERR_NONE) {
                pr_err("Data byte %zu (0x%02X) NACK\n", i, data[i]);
                my_i2c_stop(g_my_i2c_data);
                success = 0;
                break;
            }

            // 전송 간격에서 딜레이가 필요한 경우
            if(delay_us > 0) {
                udelay(delay_us);
            }
        }

        if(!success) {
            continue;
        }

        // Stop condition
        if((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
            pr_err("STOP failed\n");
            continue;
        }

        return I2C_ERR_NONE;
    }


    return err;
}

// 레지스터 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_reg_bytes(uint8_t slave_addr, uint8_t reg, const uint8_t *data, size_t len , int delay_us) {

    i2c_error_t err = I2C_ERR_NONE;

    pr_debug("[my_i2c_write_reg_bytes] %zu byte(s) from 0x%02X\n", len, slave_addr);


    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    // START
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to start\n");
        return err;
    }

    // WRITE slave addr w
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(slave_addr))) != I2C_ERR_NONE) {
        pr_err("Failed to slave addr write\n");
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // write register
    if ((err = my_i2c_write(g_my_i2c_data, reg)) != I2C_ERR_NONE) {
        pr_err("Failed to write register\n");
        my_i2c_stop(g_my_i2c_data);
        return err;
    }

    // WRITE data
    for (size_t i = 0; i < len; ++i) {
        if ((err = my_i2c_write(g_my_i2c_data, data[i])) != I2C_ERR_NONE) {
            pr_err("Failed to write data\n");
            my_i2c_stop(g_my_i2c_data);
            return err;
        }

        // 전송 간격에서 딜레이가 필요한 경우
        if(delay_us > 0) {
            udelay(delay_us);
        }
    }

    // stop
    if ((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to stop\n");
        return err;
    }

    return I2C_ERR_NONE;
}

// 레지스터 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_reg_bytes(uint8_t slave_addr, uint8_t reg, uint8_t *out_data, size_t len) {

    i2c_error_t err;
    pr_debug("[my_i2c_read_reg_bytes] %zu byte(s) from 0x%02X\n", len, slave_addr);

    if (!g_my_i2c_data || !g_my_i2c_data->ready) {
        pr_err("I2C not ready\n");
        return I2C_ERR_NOT_READY;
    }

    // write register addr
    if ((err = my_i2c_start(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to start\n");
        return err;
    }

    // write slave addr (write)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_WRITE(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        pr_err("Failed to write slave addr (w)\n");
        return err;
    }

    // write register
    if ((err = my_i2c_write(g_my_i2c_data, reg)) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        pr_err("Failed to write register\n");
        return err;
    }

    // restart
    if ((err = my_i2c_restart(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to restart\n");
        return err;
    }

    // write slave addr (read)
    if ((err = my_i2c_write(g_my_i2c_data, I2C_ADDR_READ(slave_addr))) != I2C_ERR_NONE) {
        my_i2c_stop(g_my_i2c_data);
        pr_err("Failed to write slave addr (r)\n");
        return err;
    }

    // read 
    for (size_t i = 0; i < len; ++i) {
        int ack = (i < len - 1) ? I2C_ACK : I2C_NACK;
        if ((err = my_i2c_read(g_my_i2c_data, &out_data[i], ack)) != I2C_ERR_NONE) {
            my_i2c_stop(g_my_i2c_data);
            pr_err("Failed to read\n");
            return err;
        }

        pr_debug("[READ_REG] Read[%zu] = 0x%02X\n", i, out_data[i]);
    }

    // stop
    if ((err = my_i2c_stop(g_my_i2c_data)) != I2C_ERR_NONE) {
        pr_err("Failed to stop\n");
        return err;
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

    pr_debug("state %d SDA=%d (%s) SCL=%d (%s)\n",
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


    mutex_lock(&data->lock);

    // check
    // SDA/SCL 상태 및 방향 체크
    int sda_val = gpiod_get_value(data->sda_desc);
    int scl_val = gpiod_get_value(data->scl_desc);

    bool sda_in = (gpiod_get_direction(data->sda_desc) == 1);  // 0: output
    bool scl_in = (gpiod_get_direction(data->scl_desc) == 1);

    if (!(sda_val == 1 && scl_val == 1 && sda_in && scl_in)) {
        pr_warn("Start Inavild GPIO State : SDA=%d (%s), SCL=%d (%s)\n",
                sda_val, sda_in ? "input" : "output",
                scl_val, scl_in ? "input" : "output");
    }


    data->state = I2C_STATE_START;

    // SDA/SCL HIGH
    gpiod_direction_input(data->sda_desc);
    gpiod_direction_input(data->scl_desc);

	udelay(i2c_delays_us[data->type][I2C_DELAY_START_SETUP]);

    // SDA를 LOW
    gpiod_direction_output(data->sda_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_START_HOLD]);

    // scl을 LOW
    gpiod_direction_output(data->scl_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);

    mutex_unlock(&data->lock);

    return I2C_ERR_NONE;
}

static i2c_error_t my_i2c_restart(struct my_i2c_data *data) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_RESTART;

    // 모두 HIGH로
    gpiod_direction_input(data->sda_desc);
    gpiod_direction_input(data->scl_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_RESTART_COND]);

    // SDA LOW
    gpiod_direction_output(data->sda_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_START_SETUP]);


    // SCL LOW
    gpiod_direction_output(data->scl_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);

    mutex_unlock(&data->lock);

    return I2C_ERR_NONE;
}
	
	
static i2c_error_t my_i2c_stop(struct my_i2c_data *data) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_STOP;

    // SDA LOW
    gpiod_direction_output(data->sda_desc, LOW);

    // SCL을 HIGH
    gpiod_direction_input(data->scl_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);

    // SDA를 HIGH
    gpiod_direction_input(data->sda_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_STOP_HOLD]);

    // check
    // SDA/SCL 상태 및 방향 체크
    int sda_val = gpiod_get_value(data->sda_desc);
    int scl_val = gpiod_get_value(data->scl_desc);

    bool sda_in = (gpiod_get_direction(data->sda_desc) == 1);  // 0: output
    bool scl_in = (gpiod_get_direction(data->scl_desc) == 1);

    if (!(sda_val == 1 && scl_val == 1 && sda_in && scl_in)) {
        pr_warn("STOP Inavild GPIO State : SDA=%d (%s), SCL=%d (%s)\n",
                sda_val, sda_in ? "input" : "output",
                scl_val, scl_in ? "input" : "output");
    }

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

    // scl low로 들어옴

    // scl high, sda input
    gpiod_direction_input(data->sda_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_SETUP]);


    gpiod_direction_input(data->scl_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);


    // SCL HIGH 유지 시간 동안 SDA 상태 감시
    for (int i = 0; i < I2C_ACK_TIMEOUT_US; i += i2c_delays_us[data->type][I2C_DELAY_ACK_TIMEOUT]) {
        int sda = gpiod_get_value(data->sda_desc);
        if (sda == I2C_ACK) {
            ack_received = 1;
            break;
        }
        udelay(i2c_delays_us[data->type][I2C_DELAY_ACK_TIMEOUT]);
    }

    // SCL LOW로 다시 전환
    gpiod_direction_output(data->scl_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);

    mutex_unlock(&data->lock);

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

    // SDA 설정
    if (ack == I2C_ACK) {
        gpiod_direction_output(data->sda_desc, LOW);  // ACK
    } else {
        gpiod_direction_input(data->sda_desc);        // NACK
    }

    // SCL HIGH
    gpiod_direction_input(data->scl_desc);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);

    // SCL LOW
    gpiod_direction_output(data->scl_desc, LOW);
    udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);

    // SDA 해제
    gpiod_direction_input(data->sda_desc);

    mutex_unlock(&data->lock);
    return I2C_ERR_NONE;

}


static i2c_error_t my_i2c_write(struct my_i2c_data *data, uint8_t byte) {

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    data->state = I2C_STATE_SEND_DATA;

	// write byte

    // 여기 들어올 때 scl은 LOW 상태 (start에서 low로)

	// msb to lsb 
    for (int i = 7; i >= 0; --i) {

        int bit = (byte >> i) & 0x01;


        if (bit == 0) {
            // SDA = 0 → LOW 출력
            gpiod_direction_output(data->sda_desc, LOW);
        } else {
            // SDA = 1 → input으로 해제 (풀업)
            gpiod_direction_input(data->sda_desc);
        }
        udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_SETUP]);


        // SCL = HIGH
        gpiod_direction_input(data->scl_desc);
        udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);

        // SCL = LOW
        gpiod_direction_output(data->scl_desc, LOW);
        udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);

        // // SDA 출력 설정 + bit write
        // // sda setup
        // gpiod_set_value(data->sda_desc, (byte >> i) & 0x01);
        // udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_SETUP]);
        
        // // scl low to high
        // // scl high hold time 4.0us
        // gpiod_set_value(data->scl_desc, HIGH);
        // //udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);
        // udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_HOLD]);
        // udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);


        // // scl을 low로 시작했으므로 다시 low
        // // scl low
        // // scl low hold time 4.7us
        // gpiod_set_value(data->scl_desc, LOW);
        // udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);
    }

    mutex_unlock(&data->lock);

    // // ACK check
    return my_i2c_ack(data);

    // if ((ret = my_i2c_ack(data)) != I2C_ERR_NONE) {
    //     return ret;
    // }

    // return I2C_ERR_NONE;
}

static i2c_error_t my_i2c_read(struct my_i2c_data *data, uint8_t *byte, int send_ack) {

    i2c_error_t ret = I2C_ERR_NONE;
    int read_data = 0;

    if (!data->ready) {
        return I2C_ERR_NOT_READY;
    }

    mutex_lock(&data->lock);

    // scl이 low로 들어옴 ( ack에서 low)

    // SDA input mode
    gpiod_direction_input(data->sda_desc);

    for (int i = 0; i < 8; ++i) {

        // sda setup time or scl high hold
        gpiod_direction_input(data->scl_desc);
        //udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_SETUP]);
        udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_HIGH_HOLD]);

        // SDA 상태 읽기
        read_data <<= 0x01;
        if (gpiod_get_value(data->sda_desc)) {
            read_data |= 0x01;  // SDA가 HIGH면 1
        }

        // SDA hold time or sda low hold
        gpiod_direction_output(data->scl_desc, LOW);
        //udelay(i2c_delays_us[data->type][I2C_DELAY_SDA_HOLD]);
        udelay(i2c_delays_us[data->type][I2C_DELAY_SCL_LOW_HOLD]);
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

    // 초기 상태 설정
    // SDA/SCL 초기 상태 재설정 ( input pull up )
    data->sda_desc = devm_gpiod_get(dev, I2C_SDA_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->sda_desc)) {
        pr_err("Failed to get sda_desc: %ld\n", PTR_ERR(data->sda_desc));
        return PTR_ERR(data->sda_desc);
    }

    data->scl_desc = devm_gpiod_get(dev, I2C_SCL_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->scl_desc)) {
        pr_err("Failed to get scl_desc: %ld\n", PTR_ERR(data->scl_desc));
        return PTR_ERR(data->scl_desc);
    }

    data->state = I2C_STATE_IDLE;
    slave_count = 0;

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
EXPORT_SYMBOL(my_i2c_register_device);
EXPORT_SYMBOL(my_i2c_unregister_device);
EXPORT_SYMBOL(my_i2c_debug);
EXPORT_SYMBOL(my_i2c_ping);


EXPORT_SYMBOL(my_i2c_read_byte);
EXPORT_SYMBOL(my_i2c_write_byte);
EXPORT_SYMBOL(my_i2c_write_reg_byte);
EXPORT_SYMBOL(my_i2c_read_reg_byte);

EXPORT_SYMBOL(my_i2c_read_bytes);
EXPORT_SYMBOL(my_i2c_write_bytes);
EXPORT_SYMBOL(my_i2c_write_reg_bytes);
EXPORT_SYMBOL(my_i2c_read_reg_bytes);