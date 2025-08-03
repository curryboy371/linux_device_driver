#ifndef __OLED_SSD1306_DEF_H__
#define __OLED_SSD1306_DEF_H__

/*
 * SSD1306 OLED Command Set
 */

#define OLED_LINE_HEIGHT 8
#define OLED_LINE_COUNT 8
#define OLED_WIDTH 128


 // Control Byte (I2C)
#define SSD1306_CO_CMD_SINGLE   0x00  // Co=0, D/C#=0 : 명령어 단일 전송
#define SSD1306_CO_CMD_CONT     0x80  // Co=1, D/C#=0 : 연속 명령어 전송
#define SSD1306_CO_DATA_SINGLE  0x40  // Co=0, D/C#=1 : 데이터 단일/연속 전송
#define SSD1306_CO_DATA_CONT    0xC0  // Co=1, D/C#=1 : 연속 데이터 전송 (거의 안 씀)


// Fundamental Commands
#define SSD1306_CMD_DISPLAY_OFF            0xAE  // 디스플레이 OFF
#define SSD1306_CMD_DISPLAY_ON             0xAF  // 디스플레이 ON
#define SSD1306_CMD_SET_CONTRAST           0x81  // 밝기(명암비) 설정
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME  0xA4  // RAM 내용대로 픽셀 출력
#define SSD1306_CMD_DISPLAY_ALL_ON         0xA5  // 모든 픽셀 ON (디버깅용)
#define SSD1306_CMD_NORMAL_DISPLAY         0xA6  // 일반 디스플레이 모드
#define SSD1306_CMD_INVERT_DISPLAY         0xA7  // 흑백 반전

// Scrolling (사용하지 않으면 생략 가능)
#define SSD1306_CMD_RIGHT_SCROLL         0x26
#define SSD1306_CMD_LEFT_SCROLL          0x27

// Addressing Setting
#define SSD1306_CMD_MEMORY_MODE            0x20  // Addressing mode 설정
#define SSD1306_ADDR_MODE_HORIZONTAL       0x00  // 가로 모드 (기본 권장)
#define SSD1306_ADDR_MODE_VERTICAL         0x01
#define SSD1306_ADDR_MODE_PAGE             0x02

// Hardware Configuration
#define SSD1306_CMD_SET_DISPLAY_CLOCK      0xD5  // 클럭 분주율 설정
#define SSD1306_CLOCK_DIV_DEFAULT          0x80  // Default oscillator freq

#define SSD1306_CMD_SET_MULTIPLEX          0xA8  // 멀티플렉스 비율
#define SSD1306_MULTIPLEX_64               0x3F  // 64줄 사용 (0~63)

#define SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3  // 디스플레이 수직 오프셋
#define SSD1306_DISPLAY_OFFSET_NONE        0x00

#define SSD1306_CMD_SET_START_LINE         0x40  // 시작 라인 설정 (0~63)

#define SSD1306_CMD_SEG_REMAP              0xA1  // 세그먼트 좌우 반전
#define SSD1306_CMD_COM_SCAN_DEC           0xC8  // COM 출력 스캔 방향 반전

#define SSD1306_CMD_SET_COM_PINS           0xDA  // COM 핀 구성 설정
#define SSD1306_COM_PINS_ALT_SEQ           0x12  // Alternative config

#define SSD1306_CMD_SET_PRECHARGE          0xD9  // Pre-charge 기간 설정
#define SSD1306_PRECHARGE_DEFAULT          0xF1

#define SSD1306_CMD_SET_VCOM_DETECT        0xDB  // VCOMH de-select level
#define SSD1306_VCOM_DESELECT_DEFAULT      0x40

// Charge Pump
#define SSD1306_CMD_CHARGE_PUMP            0x8D
#define SSD1306_CHARGE_PUMP_ON             0x14
#define SSD1306_CHARGE_PUMP_OFF            0x10

// Values
#define SSD1306_CONTRAST_DEFAULT           0xFF // 명암 밝기 값 ( 0x00 ~ 0xFF SSD1306_CMD_SET_CONTRAST)

// Page and Column Addressing
#define SSD1306_CMD_SET_LOW_COLUMN(col)    (0x00 | ((col) & 0x0F)) // 하위 4비트
#define SSD1306_CMD_SET_HIGH_COLUMN(col)   (0x10 | (((col) >> 4) & 0x0F)) // 상위 4비트
#define SSD1306_CMD_SET_PAGE_START(row)    (0xB0 | ((row) & 0x07)) // 페이지 주소 (0~7)

#endif // __OLED_SSD1306_DEF_H__
