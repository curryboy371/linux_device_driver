#ifndef __VS1003_DEF_H__
#define __VS1003_DEF_H__

/* VS1003 SCI Commands */
#define VS1003_WRITE_COMMAND    0x02
#define VS1003_READ_COMMAND     0x03

#define VS1003_CMD_LEN          4
#define VS1003_TEST_LEN         8


/* VS1003 SCI Register Addresses */
#define SCI_MODE                0x00    // 동작 모드 제어
#define SCI_STATUS              0x01    // VS1003 상태 정보
#define SCI_BASS                0x02    // 내장 저음/고음 향상기 제어
#define SCI_CLOCKF              0x03    // 클록 주팧수 및 배율 제어
#define SCI_DECODE_TIME         0x04    // 현재 디코딩 시간(초 단위)
#define SCI_AUDATA              0x05    // 기타 오디오 데이터 (샘플링 속도, 채널 수 등)
#define SCI_WRAM                0x06    // RAM 쓰기/읽기
#define SCI_WRAMADDR            0x07    // RAM 쓰기/읽기용 기본 주소
#define SCI_HDAT0               0x08    // 스트림 헤더 데이터 0
#define SCI_HDAT1               0x09    // 스트림 헤더 데이터 1
#define SCI_AIADDR              0x0A    // 애플리케이션 시작 주소
#define SCI_VOL                 0x0B    // 볼륨 제어
#define SCI_AICTRL0             0x0C    // 애플리케이션 제어 레지스터
#define SCI_AICTRL1             0x0D    // 애플리케이션 제어 레지스터
#define SCI_AICTRL2             0x0E    // 애플리케이션 제어 레지스터
#define SCI_AICTRL3             0x0F    // 애플리케이션 제어 레지스터

/* SCI_MODE Register Bit Definitions */
#define SM_DIFF                 0x0001    // 오디오 출력 위상을 반전
#define SM_RESET                0x0004    // 칩의 소프트웨어 리셋을 트리거
#define SM_CANCEL               0x0008    // 디코딩 취소
#define SM_PDOWN                0x0010    // 칩을 소프트웨어 파워다운 모드로 전환
#define SM_STREAM               0x0040    // 스트리밍 모드 활성화 ( 데이터 전송 속도에 따라 재생 속도 조절)
#define SM_SDINEW               0x0800    // 기본값
#define SM_ADPCM                0x1000    // ADPCM 녹음 기능을 활성화
#define SM_LINE_IN              0x4000    // 녹음 시 마이크 입력 대신 라인 입력 소스를 사용하도록 전환

#define SM_TESTS                0x0020    // TEST


/* Volume */
#define MIN_VOLUME 0xFE         // 한쪽 기준
#define MAX_VOLUME 0x00         // 한쪽 기준

  /* 선형 근사: 100%->0x00, 0%->0xFE */



/* VS1003 BUFFER SIZE */
#define VS1003_FIFO_SIZE   (64 * 1024)   // 64KB
#define VS1003_CHUNK_SIZE  32            
#define VS1003_MAX_BUFFER_SIZE 4096

/* VS1003 Hz */
#define VS1003_STREAM_HZ    3000000     // stream data write hz
#define VS1003_CMD_HZ       1000000     // cmd write hz


/* VS1003 Default Values */
#define VS1003_DEFAULT_CLOCKF   0x8800 // 기본 클록 값
#define VS1003_DEFAULT_AUDATA   0xAC45 // 기본 오디오 데이터값
#define VS1003_DEFAULT_VOLUME_PER 70   // 기본 볼륨값 - 퍼센트


typedef enum vs1003_playstate {
    PLAY_STOPPED = 0,
    PLAY_PLAYING,
    PLAY_PAUSED,
} vs1003_playstate_t;

/* inline 공통 함수 */

/* 0~100% → VS1003 VOL 바이트 */
static inline uint8_t vs1003_vol_percent_to_byte(unsigned int percent)
{
    percent = clamp_t(unsigned int, percent, 0, 100);
    if (percent >= 100)
        return MAX_VOLUME;

    return (uint8_t)((100 - percent) * MIN_VOLUME / 100);
}

/* VS1003 VOL 바이트 → 0~100% */
static inline unsigned int vs1003_vol_byte_to_percent(uint8_t v)  {

    if (v == MAX_VOLUME) return 100;
    if (v >= MIN_VOLUME) return 0;

    return (unsigned int)((MIN_VOLUME - v) * 100 / MIN_VOLUME);
}


#endif // __VS1003_DEF_H__
