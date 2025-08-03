#ifndef __LCD1602_DEF_H__
#define __LCD1602_DEF_H__

// 디바이스 정보
#define LCD_ADDR 0x27

// LCD 제어 비트 (PCF8574 통해 제어)
#define LCD_BACKLIGHT 0x08   // 백라이트 ON (P3 = 1)
#define LCD_ENABLE    0x04   // Enable 신호 (P2 = 1 → falling edge에서 latch)
#define LCD_RW        0x00   // R/W = 0: 쓰기 모드만 사용
#define LCD_RS        0x01   // RS = 1: 데이터, RS = 0: 명령

// LCD 명령어 (HD44780U 기준)
#define LCD_CMD_CLEAR_DISPLAY          0x01  // 화면 클리어, 커서 홈 위치 복귀
#define LCD_CMD_RETURN_HOME            0x02  // 커서 홈으로 이동

#define LCD_CMD_ENTRY_MODE_SET         0x06  // 커서 오른쪽으로 이동, 화면 이동 없음
#define LCD_CMD_DISPLAY_ON_CURSOR_OFF  0x0C  // 디스플레이 ON, 커서 OFF
#define LCD_CMD_DISPLAY_ON_CURSOR_ON   0x0E  // 디스플레이 ON, 커서 ON
#define LCD_CMD_DISPLAY_ON_BLINK_ON    0x0F  // 디스플레이 ON, 커서 ON + 블링크


#define LCD_CMD_DISPLAY_OFF           0x08  // 디스플레이 OFF, 커서 OFF
#define LCD_CMD_CURSOR_SHIFT_LEFT     0x10  // 커서 왼쪽으로 이동
#define LCD_CMD_CURSOR_SHIFT_RIGHT    0x14  // 커서 오른쪽으로 이동
#define LCD_CMD_DISPLAY_SHIFT_LEFT    0x18  // 화면 왼쪽으로 쉬프트
#define LCD_CMD_DISPLAY_SHIFT_RIGHT   0x1C  // 화면 오른쪽으로 쉬프트

#define LCD_CMD_FUNCTION_SET_2LINE     0x28  // 4비트 모드, 2줄, 5x8 도트
#define LCD_CMD_FUNCTION_SET_8BIT      0x30  // 8비트 모드 설정용 (초기 진입 시 사용)
#define LCD_CMD_FUNCTION_SET_4BIT      0x20  // 4비트 모드 설정 (초기 진입 마지막 단계)



#define LCD_CMD_SET_CGRAM_ADDR(addr)  (0x40 | ((addr) & 0x3F)) // CGRAM 주소 설정
#define LCD_CMD_SET_DDRAM_ADDR(addr)  (0x80 | ((addr) & 0x7F)) // DDRAM 주소 설정 (커서 위치)

// 커서 위치 계산
// 0줄이면 0x00 + col
// 1줄이면 0x40 + cal
#define LCD_LINE_ADDR(row, col)        (((row) == 0 ? 0x00 : 0x40) + (col))

// LCD 크기
#define LCD1602_LINE_WIDTH     16
#define LCD1602_TOTAL_CHARS    (LCD1602_LINE_WIDTH * 2) // 32자

// 쓰기 간격
#define WRITE_TERM_US 100

#endif // __LCD1602_DEF_H__




