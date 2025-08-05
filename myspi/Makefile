.PHONY: all clean load unload reload print

obj-m := my_spi.o

# 커널 소스 디렉토리

DRIVER_DIR ?= $(shell pwd)
KDIR ?= $(HOME)/project/linux
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

# 모듈 이름 및 .ko 파일 경로 매크로
MODULE_NAME := my_spi
MODULE_KO := $(MODULE_NAME).ko

# 기본 빌드
all:
	make -C $(KDIR) \
		M=$(DRIVER_DIR) \
		ARCH=$(ARCH) \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		modules -j12

clean:
	make -C $(KDIR) M=$(DRIVER_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

# 로드
load:
	sudo insmod $(MODULE_KO) || true

# 언로드
unload:
	sudo rmmod $(MODULE_NAME) || true

# 재로드
reload:
	$(MAKE) unload
	$(MAKE) load

print:
	@echo "KDIR: $(KDIR)"
	@echo "DRIVER_DIR: $(DRIVER_DIR)"