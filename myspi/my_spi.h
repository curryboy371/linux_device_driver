#ifndef __MY_SPI_H__
#define __MY_SPI_H__

#include <linux/types.h>
#include <linux/mutex.h>

// list 사용
#include <linux/list.h>
#include <linux/slab.h>  // kmalloc, kfree


// SPI 에러 코드
typedef enum {
    SPI_ERR_NONE = 0,             // 정상
    SPI_ERR_NON_SELECT,           // 슬레이브가 선택되지 않음
    SPI_ERR_NON_MODE,             // 선택되지 않은 모드

    SPI_ERR_UNKNOWN               // 알 수 없는 오류
} SPI_error_t;

enum spi_mode {
    SPI_MODE_NONE = 0,  // 모드 없음 (초기값)
    SPI_MODE_0,         // CPOL=0, CPHA=0
    SPI_MODE_1,         // CPOL=0, CPHA=1
    SPI_MODE_2,         // CPOL=1, CPHA=0
    SPI_MODE_3          // CPOL=1, CPHA=1
} typedef spi_mode_t;

enum spi_select {
    SPI_SLAVE_0 = 0,    // 0번 슬레이브
    SPI_SLAVE_1,        // 1번 슬레이브
    SPI_SLAVE_2,        // 2번 슬레이브

    SPI_SLAVE_MAX       // 슬레이브 수 ( 선택되지 않은 상태)
} typedef spi_select_t;

// SPI Slave data 구조체
struct my_spi_slave_data {
    spi_mode_t mode; // slave spi mode
    spi_select_t slave_id; // slave의 id
    struct mutex lock;
} typedef my_spi_slave_data_t;


// SPI messge
struct my_spi_messgae {


} typedef my_spi_messgae_t;

struct my_spi_transfer {

    const void *tx_buf;
    void *rx_buf;
    unsigned int len;



}typedef my_spi_transfer_t;


struct my_spi_data;  // 전방 선언

void my_spi_lock(struct my_spi_data* data);
void my_spi_unlock(struct my_spi_data* data);

#endif // __MY_SPI_H__

