.PHONY: all clean load unload reload print


# 모듈 목록
obj-m := myspi_sample.o

DRIVER_DIR ?= $(shell pwd)
KDIR ?= $(HOME)/project/linux
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
KBUILD_EXTRA_SYMBOLS ?= $(abspath $(DRIVER_DIR)/../myspi/Module.symvers)

# myspi 헤더 참조용
ccflags-y += -I$(src)/../myspi

# 모듈 이름 및 .ko 파일 경로 매크로
MODULE_NAME := myspi_sample
MODULE_KO := $(MODULE_NAME).ko

RAW_MODULE_NAME := myspi_sample
RAW_MODULE_KO := $(RAW_MODULE_NAME).ko

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