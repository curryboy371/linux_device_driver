// pr_debug, pr_info, pr_err 메시지 접두사 macro
#define pr_fmt(fmt) "[my_spi] " fmt

// pr_debuf 출력
#define DEBUG

#include "my_spi.h"


#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>


/*
    spi protocol을 위한 platform driver
*/


#define I2C_ADDR_WRITE(addr)  ((addr) << 0x01) // Write mode: LSB 0
#define I2C_ADDR_READ(addr)   (((addr) << 0x01) | 0x01)

#define DEVICE_NAME "my_spi"

#define SPI_SCLK_GPIO_NAME "myspi-sclk"
#define SPI_MOSI_GPIO_NAME "myspi-mosi"
#define SPI_MISO_GPIO_NAME "myspi-miso"
#define SPI_CS_GPIO_NAME   "myspi-cs"


#define MAX_SPI_SLAVES 2
#define RETRY_COUNT 3

#define HIGH   1
#define LOW    0

// 등록된 슬레이브 주소 저장
static uint8_t registered_addrs[SPI_SLAVE_MAX] = { 0 };
static int slave_count = 0;

// SPI 구조체
struct my_spi_data {
    struct gpio_desc* sclk_dec;   // 클럭
    struct gpio_desc* mosi_dec;   // 데이터 출력 (컨트롤러 → 주변장치)
    struct gpio_desc* miso_dec;   // 데이터 입력 (주변장치 → 컨트롤러)
    struct gpio_desc* cs_dec;     // 슬레이브 선택 (active-low)

    int ready; // 드라이버 준비 상태
    spi_select_t selected_slave_id; // 현재 선택된 슬레이브
    bool register_arr[SPI_SLAVE_MAX]; // slave id가 등록되었는지 여부

    spi_mode_t modes[SPI_SLAVE_MAX]; // SPI 모드
    struct mutex lock;
}typedef my_spi_data_t;


static struct my_spi_data *g_my_spi_data = NULL; // 전역 데이터 포인터

// 내부 함수 전방선언

static SPI_error_t my_spi_set_mode(my_spi_data_t* data, my_spi_slave_data_t* slave);
static SPI_error_t my_spi_set_cs(my_spi_data_t* data, my_spi_slave_data_t* slave, bool active);

static SPI_error_t my_spi_register(my_spi_data_t* data, my_spi_slave_data_t* slave);
static SPI_error_t my_spi_unregister(my_spi_data_t* data, my_spi_slave_data_t* slave);

//spi_sync() / spi_async() (실제 전송)

static SPI_error_t my_spi_sync(my_spi_data_t* data, my_spi_messgae_t* message) {

    // 사용 예시
    u8 tx_buf[4] = { 0x01, 0x01, 0x00, 0x00 }; // READ + ADDR + 2 dummy bytes
    u8 rx_buf[4] = { 0x00, 0x00, 0x00, 0x00 };

    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,  // 4바이트로 수정
    };

    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(dev->spi, &msg);

}


void my_spi_lock(struct my_spi_data* data) {
    if(data) {
        mutex_lock(&data->lock);
    }
}

void my_spi_unlock(struct my_spi_data* data) {
    if(data) {
        mutex_unlock(&data->lock);
    }
}

static SPI_error_t my_spi_register(my_spi_data_t* data, my_spi_slave_data_t* slave) {

    if(!data) {
        pr_err("my_spi_register: data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if(!slave) {
        pr_err("my_spi_register: slave is Null\n");
        return SPI_ERR_UNKNOWN;
    }

    if(slave->slave_id != SPI_SLAVE_MAX) {
        pr_err("my_spi_register: slave id is not None %d\n", slave->slave_id);
        return SPI_ERR_UNKNOWN;
    }

    if(slave->mode == SPI_MODE_NONE) {
        pr_err("my_spi_register: Invalid slave selection %d\n", slave);
        return SPI_ERR_NON_MODE;
    }


    // 남은 id 부여
    for(spi_select_t id = SPI_SLAVE_0; id <= SPI_SLAVE_MAX; ++id) {


    }


    slave->slave_id;

}

static SPI_error_t my_spi_unregister(my_spi_data_t* data, my_spi_slave_data_t* slave) {

    return SPI_ERR_NONE;
}



static SPI_error_t my_spi_set_cs(my_spi_data_t* data, my_spi_slave_data_t* slave, bool active) {
    
    if (!data) {
        pr_err("my_spi_set_cs: data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if(active) {
        // CS를 LOW : Slave 선택
        if (data->selected_slave_id == SPI_SLAVE_MAX) {
            pr_err("my_spi_set_cs: No slave selected\n");
            return SPI_ERR_NON_SELECT;
        }
        gpiod_set_value(data->cs_dec, LOW);
        data->selected_slave_id = slave->slave_id;

    } else {
        // CS를 HIGH : Idle
        gpiod_set_value(data->cs_dec, HIGH);
        data->selected_slave_id = SPI_SLAVE_MAX;
    }
    return SPI_ERR_NONE;
}

static SPI_error_t my_spi_set_mode(my_spi_data_t* data, my_spi_slave_data_t* slave) {

    if (!data) {
        pr_err("my_spi_set_mode: data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if (!slave) {
        pr_err("my_spi_set_mode: slave is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if (slave->mode == SPI_MODE_NONE) {
        pr_err("my_spi_set_mode: invalid mode\n");
        return SPI_ERR_NON_SELECT;
    }

    if (slave->slave_id == SPI_SLAVE_MAX) {
        pr_err("my_spi_set_mode: invalid slave id\n");
        return SPI_ERR_NON_MODE;
    }

    data->modes[slave->slave_id] = slave->mode;

    pr_debug("my_spi_set_mode: Set to mode %d slave %d\n", slave->mode, slave->slave_id);
    return SPI_ERR_NONE;
}

static SPI_error_t my_spi_clock_tick(my_spi_data_t* data) {

    if (!data) {
        pr_err("my_spi_clock_tick: data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if(data->selected_slave_id == SPI_SLAVE_MAX) {
        pr_err("my_spi_clock_tick: No slave selected\n");
        return SPI_ERR_NON_SELECT;
    }

    if(data->modes[data->selected_slave_id] == SPI_MODE_NONE) {
        pr_err("my_spi_clock_tick: No mode set for selected slave %d\n", data->selected_slave_id);
        return SPI_ERR_UNKNOWN;
    }

    // SPI 모드에 따라...
    data->modes[data->selected_slave_id];

}

// 플랫폼 드라이버 등록
static int my_spi_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct my_spi_data *data;
    
    pr_info("platform driver probed started\n");
    
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        return -ENOMEM;
    }
        
    // device tree gpio와 매칭
    // devm은 리소스 자동 관리
    data->sclk_dec = devm_gpiod_get(dev, SPI_SCLK_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->sclk_dec)) {
        pr_err("Failed to get sclk_dec: %ld\n", PTR_ERR(data->sclk_dec));
        return PTR_ERR(data->sclk_dec);
    }

    data->mosi_dec = devm_gpiod_get(dev, SPI_MOSI_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->mosi_dec)) {
        pr_err("Failed to get mosi_dec: %ld\n", PTR_ERR(data->mosi_dec));
        return PTR_ERR(data->mosi_dec);
    }

    data->miso_dec = devm_gpiod_get(dev, SPI_MISO_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->miso_dec)) {
        pr_err("Failed to get miso_dec: %ld\n", PTR_ERR(data->miso_dec));
        return PTR_ERR(data->miso_dec);
    }

    data->cs_dec = devm_gpiod_get(dev, SPI_CS_GPIO_NAME, GPIOD_IN);
    if (IS_ERR(data->cs_dec)) {
        pr_err("Failed to get cs_dec: %ld\n", PTR_ERR(data->cs_dec));
        return PTR_ERR(data->cs_dec);
    }

    // spi 초기 값 설정
    slave_count = 0;

    data->selected_slave_id = SPI_SLAVE_MAX;

    // 모든 슬레이브 모드를 초기화
    memset(data->modes, SPI_MODE_NONE, sizeof(data->modes)); 

    // mutex 초기화
    mutex_init(&data->lock);

    // 플랫폼 디바이스와 this driver간 data를 공유하도록 set
    platform_set_drvdata(pdev, data);
    g_my_spi_data = data;

    // ready ok
    data->ready = 1;
    pr_info("platform driver probed successfully\n");
    return 0;
}

static void my_spi_remove(struct platform_device *pdev) {

    struct my_spi_data *data = platform_get_drvdata(pdev);
    
    pr_info("my_spi_remove\n");

    if (data) {
        data->ready = 0;
        g_my_spi_data = NULL;
    }
}



/* Device Tree 매칭 테이블 */
static const struct of_device_id my_spi_of_match[] = {
    { .compatible = "chan,my-spi", }, // DTS의 compatible 문자열과 일치해야함
    { }
};
MODULE_DEVICE_TABLE(of, my_spi_of_match);

// platfomr driver 구조체
static struct platform_driver my_spi_driver = {
    .probe = my_spi_probe,
    .remove = my_spi_remove,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = my_spi_of_match,
    },
};

static int __init my_spi_init(void) {
    pr_info("driver init\n");
    return platform_driver_register(&my_spi_driver);
}

static void __exit my_spi_exit(void) {
    pr_info("driver exit\n");
    platform_driver_unregister(&my_spi_driver);
}

module_init(my_spi_init);
module_exit(my_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY SPI Platform Device Driver");

// Exported functions

EXPORT_SYMBOL(my_spi_lock);
EXPORT_SYMBOL(my_spi_unlock);