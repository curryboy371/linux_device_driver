#ifndef MY_I2C_H
#define MY_I2C_H

#include <linux/types.h>

#define HIGH 1
#define LOW  0

#define I2C_ACK  0
#define I2C_NACK 1

#define I2C_ACK_TIMEOUT_US 1000 // ACK 수신 대기 시간 us (1ms)

// I2C 상태 정의
typedef enum {
    I2C_STATE_IDLE,         // 대기 상태
    I2C_STATE_START,        // START 조건 발생 중
    I2C_STATE_SEND_ADDR,    // 주소 전송 중
    I2C_STATE_SEND_DATA,    // 데이터 전송 중
    I2C_STATE_RECV_DATA,    // 데이터 수신 중
    I2C_STATE_ACK_CHECK,    // ACK 확인 중
    I2C_STATE_MASTER_ACK,   // 마스터 ACK 전송 중
    I2C_STATE_STOP,         // STOP 조건 발생 중
    I2C_STATE_RESTART       // Repeated START 조건
} i2c_state_t;

// I2C type 정의
typedef enum  {
    I2C_DEV_GENERIC,    // 일반 모듈
    I2C_DEV_LCD1602,    // LCD 1602


    I2C_DEV_COUNT
}i2c_device_type_t;



// I2C 에러 코드
typedef enum {
    I2C_ERR_NONE = 0,             // 정상
    I2C_ERR_NO_ACK,               // ACK 수신 실패
    I2C_ERR_TIMEOUT,              // 타이밍 초과 (슬레이브 응답 없음 등)
    I2C_ERR_BUS_BUSY,             // 버스가 유휴 상태가 아님
    I2C_ERR_INVALID_ARG,          // 잘못된 파라미터
    I2C_ERR_START_CONDITION,      // START 조건 실패
    I2C_ERR_STOP_CONDITION,       // STOP 조건 실패
    I2C_ERR_LINE_STUCK,           // SDA 또는 SCL이 stuck됨
    I2C_ERR_NOT_READY,           // 준비되지 않음

    I2C_ERR_OVERFLOW,             // 등록 가능 주소 없음
    I2C_ERR_DUPLICATE,            // 이미 등록된 주소
    I2C_INAVLID_ADDR,             // 등록되지 않은 주소
    
    I2C_ERR_UNKNOWN               // 알 수 없는 오류
} i2c_error_t;

typedef enum {
    I2C_DELAY_START_SETUP,       // START condition setup time - SCL이 HIGH인 동안 SDA의 HIGH에서 LOW 전환이 안정적으로 유지되어야 하는 시간.
    I2C_DELAY_START_HOLD,       // START condition hold time -  첫 번째 클럭 펄스가 생성되기 전 이 기간 동안 SDA가 LOW로 유지되어야 합니다.
    I2C_DELAY_STOP_SETUP,       // STOP condition setup time - SCL이 HIGH인 동안 SDA의 LOW에서 HIGH 전환이 안정적으로 유지되어야 하는 시간.
    I2C_DELAY_STOP_HOLD,       // STOP condition hold time - 

    I2C_DELAY_SCL_LOW_HOLD,     // SCL LOW hold time - SCL 신호가 LOW 레벨로 유지되어야 하는 최소 시간
    I2C_DELAY_SCL_HIGH_HOLD,    // SCL HIGH hold time - SCL 신호가 HIGH 레벨로 유지되어야 하는 최소 시간
    I2C_DELAY_SDA_SETUP,        // SDA data setup time - SCL의 상승 에지 이전에 SDA 데이터가 안정적으로 설정되어야 하는 시간.
    I2C_DELAY_SDA_HOLD,         // SDA data hold time - SCL의 하강 에지 이후 SDA 데이터가 안정적으로 유지되어야 하는 시간.

    I2C_DELAY_RESTART_COND,     // Restart condition bus free
    I2C_DELAY_ACK_TIMEOUT,      // ACK polling loop delay
    I2C_DELAY_TEMP,             // Temp delay (1ms)

    I2C_DELAY_COUNT             // 배열 크기 확인용
} i2c_delay_t;

// Register a new I2C slave device
i2c_error_t my_i2c_register_device(uint8_t slave_addr, i2c_device_type_t type);

// Unregister an I2C slave device
i2c_error_t my_i2c_unregister_device(uint8_t slave_addr);


// my_i2c driver ping
i2c_error_t my_i2c_ping(uint8_t slave_addr);


//// single byte 함수 

// slave 주소에서 바이트 읽기
i2c_error_t my_i2c_read_byte(uint8_t slave_addr, uint8_t *out_data);

// slave 주소에 바이트 쓰기
i2c_error_t my_i2c_write_byte(uint8_t addr, const uint8_t data);

// 레지스터 주소에 바이트 쓰기
i2c_error_t my_i2c_write_reg_byte(uint8_t slave_addr, uint8_t reg, const uint8_t data);

// 레지스터 주소에서 바이트 읽기
i2c_error_t my_i2c_read_reg_byte(uint8_t slave_addr, uint8_t reg, uint8_t* out_data);




//// multi bytes 함수 
// slave 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_bytes(uint8_t slave_addr, uint8_t *out_data, size_t len);

// slave 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_bytes(uint8_t addr, const uint8_t *data, size_t len, int delay_us);

// 레지스터 주소에 여러 바이트 쓰기
i2c_error_t my_i2c_write_reg_bytes(uint8_t slave_addr, uint8_t reg, const uint8_t *data, size_t len, int delay_us);

// 레지스터 주소에서 여러 바이트 읽기
i2c_error_t my_i2c_read_reg_bytes(uint8_t slave_addr, uint8_t reg, uint8_t *out_data, size_t len);

void my_i2c_lock(void);
void my_i2c_unlock(void);

// debug
void my_i2c_debug(void);


#endif // MY_I2C_H
