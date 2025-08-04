#ifndef __VS1003_DEF_H__
#define __VS1003_DEF_H__



#define VS1003_WRITE_COMMAND	0x02
#define VS1003_READ_COMMAND		0x03

/* VS1003 Register */
#define SPI_MODE        	0x00
#define SPI_STATUS      	0x01
#define SPI_BASS        	0x02
#define SPI_CLOCKF      	0x03
#define SPI_DECODE_TIME 	0x04
#define SPI_AUDATA      	0x05
#define SPI_WRAM        	0x06
#define SPI_WRAMADDR    	0x07
#define SPI_HDAT0       	0x08
#define SPI_HDAT1       	0x09
#define SPI_AIADDR      	0x0a
#define SPI_VOL         	0x0b
#define SPI_AICTRL0     	0x0c
#define SPI_AICTRL1     	0x0d
#define SPI_AICTRL2     	0x0e
#define SPI_AICTRL3     	0x0f
#define SM_DIFF         	0x01
#define SM_JUMP         	0x02
#define SM_RESET        	0x04
#define SM_OUTOFWAV     	0x08
#define SM_PDOWN        	0x10
#define SM_TESTS        	0x20
#define SM_STREAM       	0x40
#define SM_PLUSV        	0x80
#define SM_DACT         	0x100
#define SM_SDIORD       	0x200
#define SM_SDISHARE     	0x400
#define SM_SDINEW       	0x800
#define SM_ADPCM        	0x1000
#define SM_ADPCM_HP     	0x2000

#endif // __VS1003_DEF_H__
