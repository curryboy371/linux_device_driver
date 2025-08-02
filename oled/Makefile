.PHONY: all clean load unload reload print

DRIVER_DIR ?= $(shell pwd)
KDIR ?= $(HOME)/project/linux
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
KBUILD_EXTRA_SYMBOLS ?= $(abspath $(DRIVER_DIR)/../myi2c/Module.symvers)

# myi2c 헤더 참조용
ccflags-y += -I$(src)/../myi2c


# 메인 OLED 문자 디바이스 드라이버
MODULE_NAME := oled_driver
MODULE_KO := $(MODULE_NAME).ko
obj-m += $(MODULE_NAME).o
oled_driver-objs := oled_ssd1306.o oled_font.o

# RAW OLED 드라이버
RAW_MODULE_NAME := oled_driver_raw
RAW_MODULE_KO := $(RAW_MODULE_NAME).ko
obj-m += $(RAW_MODULE_NAME).o
oled_driver_raw-objs := oled_ssd1306_raw.o oled_font.o 

all:
	make -C $(KDIR) \
		M=$(DRIVER_DIR) \
		ARCH=$(ARCH) \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS) \
		modules -j12

clean:
	make -C $(KDIR) M=$(DRIVER_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

# 로드 조건부 (raw=0,1,all)
load:
ifeq ($(raw),1)
	sudo insmod $(RAW_MODULE_KO) || true
else ifeq ($(raw),all)
	sudo insmod $(MODULE_KO) || true
	sudo insmod $(RAW_MODULE_KO) || true
else
	sudo insmod $(MODULE_KO) || true
endif

# 언로드 조건부
unload:
ifeq ($(raw),1)
	sudo rmmod $(RAW_MODULE_NAME) || true
else ifeq ($(raw),all)
	sudo rmmod $(RAW_MODULE_NAME) || true
	sudo rmmod $(MODULE_NAME) || true
else
	sudo rmmod $(MODULE_NAME) || true
endif

# 재로드
reload:
	$(MAKE) unload raw=$(raw)
	$(MAKE) load raw=$(raw)

print:
	@echo "KDIR: $(KDIR)"
	@echo "DRIVER_DIR: $(DRIVER_DIR)"