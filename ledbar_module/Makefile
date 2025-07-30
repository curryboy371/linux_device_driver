obj-m := ledbar_module.o

KDIR := $(HOME)/project/linux
PWD  := $(shell pwd)

ARCH ?= x86_64
CROSS_COMPILE ?=
ifeq ($(CROSS),1)
    ARCH := arm64
    CROSS_COMPILE := aarch64-linux-gnu-
endif

# 헤더 경로
ccflags-y += -I$(PWD)/../rotary

# rotary 심볼을 찾기 위해 Module.symvers 참조
#KBUILD_EXTRA_SYMBOLS := $(shell pwd)/../rotary/Module.symvers
KBUILD_EXTRA_SYMBOLS := /home/pi/linux_device_driver/rotary/Module.symvers


all:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
