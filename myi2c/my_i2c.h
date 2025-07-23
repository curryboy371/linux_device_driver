#ifndef MY_I2C_H
#define MY_I2C_H

#include <linux/types.h>


#define DEVICE_NAME "my_i2c"



// TODO temp pin
#define I2C_SDA_GPIO    10
#define I2C_SCL_GPIO    11

#define MAX_BUF_SIZE 32



#define HIGH 1
#define LOW  0

#define I2C_ACK_TIMEOUT_US 1000 // ACK 수신 대기 시간 us (1ms)


typedef enum {
    I2C_STATE_IDLE,         // 대기 상태
    I2C_STATE_START,        // START 조건 발생 중
    I2C_STATE_SEND_ADDR,    // 주소 전송 중
    I2C_STATE_SEND_DATA,    // 데이터 전송 중
    I2C_STATE_RECV_DATA,    // 데이터 수신 중
    I2C_STATE_ACK_CHECK,    // ACK 확인 중
    I2C_STATE_STOP,         // STOP 조건 발생 중
    I2C_STATE_RESTART       // Repeated START 조건
} i2c_state_t;

typedef enum {
    I2C_ERR_NONE = 0,             // 정상
    I2C_ERR_NO_ACK,               // ACK 수신 실패
    I2C_ERR_TIMEOUT,              // 타이밍 초과 (슬레이브 응답 없음 등)
    I2C_ERR_BUS_BUSY,             // 버스가 유휴 상태가 아님
    I2C_ERR_INVALID_ARG,          // 잘못된 파라미터
    I2C_ERR_START_CONDITION,      // START 조건 실패
    I2C_ERR_STOP_CONDITION,       // STOP 조건 실패
    I2C_ERR_LINE_STUCK,           // SDA 또는 SCL이 stuck됨
    I2C_ERR_UNKNOWN               // 알 수 없는 오류
} i2c_error_t;

i2c_error_t my_i2c_master_ack(int ack);
i2c_error_t my_i2c_start(void);
i2c_error_t my_i2c_stop(void);
i2c_error_t my_i2c_ack(void);
i2c_error_t my_i2c_read_byte(uint8_t *byte, int ack);
i2c_error_t my_i2c_write_byte(uint8_t data);    


#endif // MY_I2C_H
