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

// bitmap
#include <linux/bitmap.h>


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

// bitmap 사용

// SPI 구조체
typedef struct my_spi_data {
    struct gpio_desc* sclk_dec;   // 클럭
    struct gpio_desc* mosi_dec;   // 데이터 출력 (컨트롤러 → 주변장치)
    struct gpio_desc* miso_dec;   // 데이터 입력 (주변장치 → 컨트롤러)
    struct gpio_desc* cs_dec;     // 슬레이브 선택 (active-low)

    int ready; // 드라이버 준비 상태

    // bitmap 사용 내부적으로 bit index로 관리
    DECLARE_BITMAP(register_bitmap, SPI_SLAVE_MAX);
    int slave_count;                 // 현재 등록중 슬레이브 수

    spi_select_t selected_slave_id; // 현재 선택된 슬레이브
    struct mutex lock;
} my_spi_data_t;



static struct my_spi_data *g_my_spi_data = NULL; // 전역 데이터 포인터

// spi mode function ptr 
//typedef void (*my_spi_mode_func_t(my_spi_data_t data, uint8_t tx_byte, uint8_t *rx_byte);
typedef void (*my_spi_transfer_func_t)(my_spi_data_t *data, u8 tx, u8 *rx);

static void my_spi_transfer_byte_mode0(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte);
static void my_spi_transfer_byte_mode1(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte);
static void my_spi_transfer_byte_mode2(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte);
static void my_spi_transfer_byte_mode3(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte);

static my_spi_transfer_func_t my_spi_func_arr[SPI_MODE_MAX] = {
    [SPI_MODE_0] = my_spi_transfer_byte_mode0,
    [SPI_MODE_1] = my_spi_transfer_byte_mode1,
    [SPI_MODE_2] = my_spi_transfer_byte_mode2,
    [SPI_MODE_3] = my_spi_transfer_byte_mode3,
};

// 내부 함수 전방선언
static SPI_error_t my_spi_set_cs(spi_select_t slave_id, bool active);


SPI_error_t my_spi_sync(my_spi_slave_data_t* slave_data, my_spi_message_t* message);

static void my_spi_message_init(my_spi_message_t *m);
static void my_spi_message_add_tail(my_spi_transfer_t* transfer, my_spi_message_t* msg);

//spi_sync() / spi_async() (실제 전송)
SPI_error_t my_spi_sync(my_spi_slave_data_t* slave_data, my_spi_message_t* message) {

    if(!slave_data) {
        pr_err("my_spi_sync: slave_data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if(slave_data->slave_id == SPI_SLAVE_MAX) {
        pr_err("my_spi_sync: non select slave\n");
        return SPI_ERR_NON_SELECT;
    } 

    if(slave_data->mode == SPI_MODE_MAX) {
        pr_err("my_spi_sync: non select slave mode\n");
        return SPI_ERR_NON_MODE;
    } 

    my_spi_lock(g_my_spi_data);

    // CS 활성화
    my_spi_set_cs(slave_data->slave_id, LOW);
    udelay(SPI_DELAY_US);

    // 리스트 초기화
    my_spi_message_init(message);

    // 사용 예시
    u8 tx_buf[4] = { 0x01, 0x02, 0x03, 0x04 }; // READ + ADDR + 2 dummy bytes
    u8 rx_buf[4] = { 0x00, 0x00, 0x00, 0x00 };

    // spi_transfer instance
    my_spi_transfer_t xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 4,
    };

    // 리스트 추가( 추가노드 xfer, 리스트 헤드msg)
    my_spi_message_add_tail(&xfer, message);

    // 순회시 현재 노드를 가리킬 포인터
    my_spi_transfer_t* xfer_iter = NULL;
    list_for_each_entry(xfer_iter, &message->transfers, transfer_list) {

        if(xfer_iter) {
            for (size_t i = 0; i < xfer_iter->len; i++) {
                u8 tx = xfer_iter->tx_buf ? ((u8*)xfer_iter->tx_buf)[i] : 0xFF;
                u8 rx = 0;

                // 로그 출력
                pr_info("tx[%zu] = 0x%02X -> rx[%zu] = 0x%02X\n", i, tx, i, rx);

                // 현재 모드에 맞는 transfer 함수 호출
                my_spi_func_arr[slave_data->mode](g_my_spi_data, tx, &rx);

                if (xfer_iter->rx_buf) {
                    ((u8*)xfer_iter->rx_buf)[i] = rx;
                }
            }
        }
    }

    // CS 비활성화
    udelay(SPI_DELAY_US);
    my_spi_set_cs(slave_data->slave_id, HIGH);

    my_spi_unlock(g_my_spi_data);

    return SPI_ERR_NONE;
}


static void my_spi_message_init(my_spi_message_t* msg) {

    memset(msg, 0, sizeof(*msg));
    INIT_LIST_HEAD(&msg->transfers);
    INIT_LIST_HEAD(&msg->resources);
}


static void my_spi_message_add_tail(my_spi_transfer_t* transfer, my_spi_message_t* msg) {
    list_add_tail(&transfer->transfer_list, &msg->transfers);
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

void my_spi_slave_init(my_spi_slave_data_t* slave) {
    
    if (!slave) {
        pr_err("my_spi_slave_init: NULL or slave\n");
        return;
    }

    slave->slave_id = SPI_SLAVE_MAX;
    slave->mode = SPI_MODE_MAX;
}


SPI_error_t my_spi_register(my_spi_slave_data_t* slave) {

    if (!slave) {
        pr_err("my_spi_register: NULL or slave\n");
        return SPI_ERR_UNKNOWN;
    }

    if (slave->slave_id != SPI_SLAVE_MAX) {
        pr_err("my_spi_register: slave already registered (%d)\n", slave->slave_id);
        return SPI_ERR_UNKNOWN;
    }

    if (slave->mode == SPI_MODE_MAX) {
        pr_err("my_spi_register: Invalid mode %d\n", slave->mode);
        return SPI_ERR_NON_MODE;
    }

    mutex_lock(&g_my_spi_data->lock);

    // 비어있는 슬롯 검색
    int id = find_first_zero_bit(g_my_spi_data->register_bitmap, SPI_SLAVE_MAX);
    if (id >= SPI_SLAVE_MAX) {
        pr_err("my_spi_register: No available slot\n");
        mutex_unlock(&g_my_spi_data->lock);
        return SPI_ERR_UNKNOWN;
    }

    set_bit(id, g_my_spi_data->register_bitmap);
    slave->slave_id = (spi_select_t)id;
    g_my_spi_data->slave_count++;

    pr_info("my_spi_register: Registered slave id=%d mode=%d\n", id, slave->mode);

    mutex_unlock(&g_my_spi_data->lock);
    return SPI_ERR_UNKNOWN;
}

SPI_error_t my_spi_unregister(my_spi_slave_data_t* slave) {

    if (!slave) {
        pr_err("my_spi_unregister: NULL slave\n");
        return SPI_ERR_UNKNOWN;
    }

    if (slave->slave_id >= SPI_SLAVE_MAX) {
        pr_err("my_spi_unregister: Invalid slave_id=%d\n", slave->slave_id);
        return SPI_ERR_UNKNOWN;
    }

    mutex_lock(&g_my_spi_data->lock);

    if (!test_bit(slave->slave_id, g_my_spi_data->register_bitmap)) {
        pr_err("my_spi_unregister: Slave id=%d not registered\n", slave->slave_id);
        mutex_unlock(&g_my_spi_data->lock);
        return SPI_ERR_UNKNOWN;
    }

    clear_bit(slave->slave_id, g_my_spi_data->register_bitmap);
    g_my_spi_data->slave_count--;
    pr_info("my_spi_unregister: Unregistered slave id=%d\n", slave->slave_id);

    slave->slave_id = SPI_SLAVE_MAX;

    mutex_unlock(&g_my_spi_data->lock);
    return SPI_ERR_NONE;
}


static SPI_error_t my_spi_set_cs(spi_select_t slave_id, bool active) {
    
    if (!g_my_spi_data) {
        pr_err("my_spi_set_cs: data is NULL\n");
        return SPI_ERR_UNKNOWN;
    }

    if(slave_id == SPI_SLAVE_MAX) {
        pr_err("my_spi_set_cs: Invalid slave ID\n");
        return SPI_ERR_INVALID_ID;
    }

    gpiod_set_value(g_my_spi_data->cs_dec, active);

    if(!active) {
        g_my_spi_data->selected_slave_id = slave_id;
    }
    else {
        g_my_spi_data->selected_slave_id = SPI_SLAVE_MAX;
    }
    return SPI_ERR_NONE;
}

// SPI MODE 0
static void my_spi_transfer_byte_mode0(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte) {

    if(!data) {
        pr_err("my_spi_gpio_send_recv_byte: data is NULL\n");
        return;
    }

    uint8_t read = 0;

    // 클럭 기본 LOW
    // 출력 후 상승 에지에서 샘플링

    // 초기 클럭상태 설정
    gpiod_set_value(data->sclk_dec, LOW);

    for (int i = 7; i >= 0; i--) {
        // MOSI 출력 (tx bit) 
        // MSB to LSB
        gpiod_set_value(data->mosi_dec, (tx_byte >> i) & 0x1);
        udelay(SPI_DELAY_US);

        // CLK 상승 (샘플링은 상승 시)
        gpiod_set_value(data->sclk_dec, HIGH);
        udelay(SPI_DELAY_US);

        // MISO 읽기
        int bit = gpiod_get_value(data->miso_dec);
        read |= (bit << i);

        // CLK 하강
        gpiod_set_value(data->sclk_dec, LOW);
        udelay(SPI_DELAY_US);
    }

    if (rx_byte)
        *rx_byte = read;
}

// SPI MODE 1
static void my_spi_transfer_byte_mode1(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte) {

    if(!data) {
        pr_err("my_spi_gpio_send_recv_byte: data is NULL\n");
        return;
    }

    uint8_t read = 0;

    // 클럭 기본 LOW
    // 클럭이 한 번 움직인 후 하강 에지에서 샘플링

    // 초기 클럭상태 설정
    gpiod_set_value(data->sclk_dec, LOW);


    // 하강 엣지 샘플링을 위해 HIGH로
    gpiod_set_value(data->sclk_dec, HIGH);

    for (int i = 7; i >= 0; i--) {

        // MOSI 출력 (tx bit) 
        // MSB to LSB
        gpiod_set_value(data->mosi_dec, (tx_byte >> i) & 0x1);
        udelay(SPI_DELAY_US);


        gpiod_set_value(data->sclk_dec, HIGH);
        udelay(SPI_DELAY_US);

        // 샘플링
        gpiod_set_value(data->sclk_dec, LOW);
        udelay(SPI_DELAY_US);

        // MISO 읽기
        int bit = gpiod_get_value(data->miso_dec);
        read |= (bit << i);
    }

    if (rx_byte)
        *rx_byte = read;
}

// SPI MODE 2
static void my_spi_transfer_byte_mode2(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte) {

    if(!data) {
        pr_err("my_spi_gpio_send_recv_byte: data is NULL\n");
        return;
    }

    uint8_t read = 0;

    // 클럭 기본 HIGH
    // 출력 후 하강 에지에서 샘플링

    // 초기 클럭상태 설정
    gpiod_set_value(data->sclk_dec, HIGH);

    
    for (int i = 7; i >= 0; i--) {

        // MOSI 출력 (tx bit) 
        // MSB to LSB
        gpiod_set_value(data->mosi_dec, (tx_byte >> i) & 0x1);
        udelay(SPI_DELAY_US);

        // CLK 하강 (샘플링은 하강에서)
        gpiod_set_value(data->sclk_dec, LOW);
        udelay(SPI_DELAY_US);

        // MISO 읽기
        int bit = gpiod_get_value(data->miso_dec);
        read |= (bit << i);

        gpiod_set_value(data->sclk_dec, HIGH);
        udelay(SPI_DELAY_US);
    }

    if (rx_byte)
        *rx_byte = read;
}


// SPI MODE 3
static void my_spi_transfer_byte_mode3(my_spi_data_t* data, uint8_t tx_byte, uint8_t *rx_byte) {

    if(!data) {
        pr_err("my_spi_gpio_send_recv_byte: data is NULL\n");
        return;
    }

    uint8_t read = 0;

    // 클럭 기본 HIGH
    gpiod_set_value(data->sclk_dec, HIGH);
    udelay(SPI_DELAY_US);

    // 상승 에지에서 샘플링

    for (int i = 7; i >= 0; i--) {
        
        gpiod_set_value(data->sclk_dec, LOW);
        udelay(SPI_DELAY_US);

        gpiod_set_value(data->mosi_dec, (tx_byte >> i) & 0x1);
        udelay(SPI_DELAY_US);

        // 샘플링
        gpiod_set_value(data->sclk_dec, HIGH);
        udelay(SPI_DELAY_US);

        int bit = gpiod_get_value(data->miso_dec);
        read |= (bit << i);
    }

    if (rx_byte)
        *rx_byte = read;
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


    return SPI_ERR_NON_MODE;
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
    data->selected_slave_id = SPI_SLAVE_MAX;
    data->slave_count = 0;


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
EXPORT_SYMBOL(my_spi_sync);

EXPORT_SYMBOL(my_spi_slave_init);
EXPORT_SYMBOL(my_spi_register);
EXPORT_SYMBOL(my_spi_unregister);
