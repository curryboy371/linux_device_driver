#ifndef __MY_SPI_H__
#define __MY_SPI_H__

#include <linux/types.h>
#include <linux/mutex.h>

// list 사용
#include <linux/list.h>
#include <linux/slab.h>  // kmalloc, kfree

#define SPI_DELAY_US 1



// SPI 에러 코드
typedef enum {
    SPI_ERR_NONE = 0,             // 정상
    SPI_ERR_INVALID_ID,           // 슬레이브가 ID가 유효하지 않음
    SPI_ERR_NON_SELECT,           // 슬레이브가 선택되지 않음
    SPI_ERR_NON_MODE,             // 선택되지 않은 모드

    SPI_ERR_UNKNOWN               // 알 수 없는 오류
} SPI_error_t;

typedef enum spi_mode {
    SPI_MODE_0 = 0,     // CPOL=0, CPHA=0
    SPI_MODE_1,         // CPOL=0, CPHA=1
    SPI_MODE_2,         // CPOL=1, CPHA=0
    SPI_MODE_3,         // CPOL=1, CPHA=1
    SPI_MODE_MAX       // 모드 없음 (초기값)
} spi_mode_t;

typedef enum spi_select {
    SPI_SLAVE_0 = 0,    // 0번 슬레이브
    SPI_SLAVE_1,        // 1번 슬레이브
    SPI_SLAVE_2,        // 2번 슬레이브

    SPI_SLAVE_MAX       // 슬레이브 수 ( 선택되지 않은 상태)
} spi_select_t;

// SPI Slave data 구조체
typedef struct my_spi_slave_data {
    spi_mode_t mode; // slave spi mode
    spi_select_t slave_id; // slave의 id
    struct mutex lock;
}my_spi_slave_data_t;


// SPI messge
typedef struct my_spi_messgae {
    // transfer들의 리스트 헤드
    struct list_head    transfers;

    /* SPI 메시지가 처리될 때 spi_res 리소스 목록 */
    struct list_head        resources;
    // struct spi_device   *spi;

}my_spi_message_t;

typedef struct my_spi_transfer {

    const void *tx_buf;
    void *rx_buf;
    unsigned int len;

    // 리스트에 연결되는 노드
    struct list_head transfer_list;

}my_spi_transfer_t;


struct my_spi_data;  // 전방 선언

void my_spi_slave_init(my_spi_slave_data_t* slave);
SPI_error_t my_spi_register(my_spi_slave_data_t* slave);
SPI_error_t my_spi_unregister(my_spi_slave_data_t* slave);


void my_spi_lock(struct my_spi_data* data);
void my_spi_unlock(struct my_spi_data* data);

SPI_error_t my_spi_sync(my_spi_slave_data_t* slave_data, my_spi_transfer_t* xfer);
SPI_error_t my_spi_ping_loopback(my_spi_slave_data_t* slave_data);



#endif // __MY_SPI_H__

