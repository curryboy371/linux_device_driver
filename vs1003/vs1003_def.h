#ifndef __VS1003_DEF_H__
#define __VS1003_DEF_H__

/* VS1003 SCI Commands */
#define VS1003_WRITE_COMMAND    0x02
#define VS1003_READ_COMMAND     0x03

#define VS1003_CMD_LEN          4
#define VS1003_TEST_LEN         8

/* VS1003 SCI Register Addresses */
#define SCI_MODE                0x00
#define SCI_STATUS              0x01
#define SCI_BASS                0x02
#define SCI_CLOCKF              0x03
#define SCI_DECODE_TIME         0x04
#define SCI_AUDATA              0x05
#define SCI_WRAM                0x06
#define SCI_WRAMADDR            0x07
#define SCI_HDAT0               0x08
#define SCI_HDAT1               0x09
#define SCI_AIADDR              0x0A
#define SCI_VOL                 0x0B
#define SCI_AICTRL0             0x0C
#define SCI_AICTRL1             0x0D
#define SCI_AICTRL2             0x0E
#define SCI_AICTRL3             0x0F

/* SCI_MODE Register Bit Definitions */
#define SM_DIFF                 0x0001
#define SM_JUMP                 0x0002
#define SM_RESET                0x0004
#define SM_OUTOFWAV             0x0008
#define SM_CANCEL               0x0008    // alternate name used in Linux driver
#define SM_PDOWN                0x0010
#define SM_TESTS                0x0020
#define SM_STREAM               0x0040
#define SM_PLUSV                0x0080
#define SM_DACT                 0x0100
#define SM_SDIORD               0x0200
#define SM_SDISHARE             0x0400
#define SM_SDINEW               0x0800
#define SM_ADPCM                0x1000
#define SM_ADPCM_HP             0x2000
#define SM_LINE_IN              0x4000    // Optional if used


#endif // __VS1003_DEF_H__
