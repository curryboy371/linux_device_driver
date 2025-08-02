#ifndef BMP180_RAW_H
#define BMP180_RAW_H

#include <linux/types.h>

//  보정 테이블 레지스터 주소 0xAA~0xBF
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


#endif // BMP180_RAW_H