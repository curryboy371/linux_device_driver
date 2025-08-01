# 커널 빌드 경로 및 크로스 컴파일 설정
KDIR := $(HOME)/project/linux
ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

# ko 파일 경로 정의
MYI2C_KO := myi2c/my_i2c.ko
BMP180_RAW_KO := bmp180_raw/bmp180_raw.ko
LCD1602_RAW_KO := lcd1602_raw/lcd1602_raw.ko


ROTARY_KO := rotary/rotary_module.ko
LEDBAR_KO := ledbar_module/ledbar_module.ko


.PHONY: all clean load unload reload

# 전체 빌드
all:
	$(MAKE) -C myi2c CROSS=1
	$(MAKE) -C bmp180_raw CROSS=1
	$(MAKE) -C lcd1602_raw CROSS=1


#	$(MAKE) -C rotary CROSS=1
#	$(MAKE) -C ledbar_module CROSS=1

# 전체 클린
clean:
	$(MAKE) -C myi2c clean
	$(MAKE) -C bmp180_raw clean
	$(MAKE) -C lcd1602_raw clean


	$(MAKE) -C rotary clean
	$(MAKE) -C ledbar_module clean

# 전체 모듈 로드
load:
	sudo insmod $(MYI2C_KO)
	
	sudo insmod $(BMP180_RAW_KO)
	sudo insmod $(LCD1602_RAW_KO)

#	sudo insmod $(ROTARY_KO)
#	sudo insmod $(LEDBAR_KO)
	dmesg | tail -n 20

# 전체 모듈 언로드
unload:
	sudo rmmod bmp180_raw || true
	sudo rmmod lcd1602_raw || true

	sudo rmmod my_i2c || true

#	sudo rmmod ledbar_module || true
#	sudo rmmod rotary_module || true
	dmesg | tail -n 20

# 언로드 후 다시 로드
reload: unload load
