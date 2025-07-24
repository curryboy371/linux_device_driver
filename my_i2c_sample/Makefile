obj-m := my_i2c_sample.o

# make 일반 컴파일
# make CROSS=1 크로스 컴파일

KDIR := $(HOME)/project/linux
PWD  := $(shell pwd)

# myi2c 헤더 파일 경로
ccflags-y += -I$(src)/../myi2c

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
