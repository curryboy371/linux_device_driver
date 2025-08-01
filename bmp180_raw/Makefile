obj-m := bmp180_raw.o

# make 일반 컴파일
# make CROSS=1 크로스 컴파일

KDIR := $(HOME)/project/linux
PWD  := $(shell pwd)


# myi2c 헤더 파일 경로
ccflags-y += -I$(src)/../myi2c

# i2c 심볼을 찾기 위해 Module.symvers 참조
KBUILD_EXTRA_SYMBOLS := /home/pi/linux_device_driver/myi2c/Module.symvers


# 기본값: x86_64
ARCH ?= x86_64
CROSS_COMPILE ?=

ifeq ($(CROSS),1)
    ARCH := arm64
    CROSS_COMPILE := aarch64-linux-gnu-
endif

all:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules -j12

clean:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
