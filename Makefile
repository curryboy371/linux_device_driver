.PHONY: all clean load unload reload push deploy print

# 커널 빌드 경로 및 크로스 컴파일 설정
KDIR := $(HOME)/project/linux
ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

BASE := $(shell pwd)
I2C_SYMBOLS := $(BASE)/myi2c/Module.symvers

# 대상 장비 정보

#TARGET_IP := pi@192.168.219.106
TARGET_IP := pi@10.10.16.31
TARGET_DIR := /home/pi/linux_device_driver


# defualt 1 (raw 빌드)
raw ?= 1



# 전체 빌드
all:
	$(MAKE) -C myspi \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/myspi

	$(MAKE) -C vs1003 \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/vs1003


	$(MAKE) -C myi2c \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/myi2c

	$(MAKE) -C bmp180 \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/bmp180 \
		KBUILD_EXTRA_SYMBOLS=$(I2C_SYMBOLS)

	$(MAKE) -C lcd1602 \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/lcd1602 \
		KBUILD_EXTRA_SYMBOLS=$(I2C_SYMBOLS)


	$(MAKE) -C oled \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/oled \
		KBUILD_EXTRA_SYMBOLS=$(I2C_SYMBOLS)


# 전체 클린
clean:
	$(MAKE) -C myspi clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/myspi


	$(MAKE) -C myi2c clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/myi2c

	$(MAKE) -C bmp180 clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/bmp180

	$(MAKE) -C lcd1602 clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/lcd1602


	$(MAKE) -C oled clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/oled


	$(MAKE) -C vs1003 clean \
		KDIR=$(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		DRIVER_DIR=$(BASE)/vs1003


# 전체 모듈 로드 (raw 옵션 전달)
load:
	$(MAKE) -C myi2c load
	$(MAKE) -C bmp180 load raw=$(raw)
	$(MAKE) -C lcd1602 load raw=$(raw)
	$(MAKE) -C oled load raw=$(raw)

	$(MAKE) -C myspi load
	dmesg | tail -n 20

# 전체 모듈 언로드 (raw 옵션 전달)
unload:
	$(MAKE) -C lcd1602 unload raw=$(raw)
	$(MAKE) -C bmp180 unload raw=$(raw)
	$(MAKE) -C oled unload raw=$(raw)

	$(MAKE) -C myi2c unload

	$(MAKE) -C myspi unload
	dmesg | tail -n 20

# 언로드 후 다시 로드
reload:
	$(MAKE) unload raw=$(raw)
	$(MAKE) load raw=$(raw)

# 라즈베리 파이 전송
push:
	scp -r . $(TARGET_IP):$(TARGET_DIR)

# 빌드 후 전송
deploy: all push


print:
	@echo "KDIR = $(KDIR)"
	@echo "BASE = $(BASE)"
	@echo "I2C_SYMBOLS = $(I2C_SYMBOLS)"

	@echo "TARGET_IP = $(TARGET_IP)"
	@echo "TARGET_DIR = $(TARGET_DIR)"