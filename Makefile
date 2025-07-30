KDIR := $(HOME)/project/linux
ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

# ROTARY + LEDBAR Make
ROTARY_KO := rotary/rotary_module.ko
LEDBAR_KO := ledbar_module/ledbar_module.ko

.PHONY: all clean load unload reload

all:
	$(MAKE) -C rotary CROSS=1
	$(MAKE) -C ledbar_module CROSS=1

clean:
	$(MAKE) -C rotary clean CROSS=1
	$(MAKE) -C ledbar_module clean CROSS=1

load:
	sudo insmod $(ROTARY_KO)
	sudo insmod $(LEDBAR_KO)
	dmesg | tail -n 20

unload:
	sudo rmmod ledbar_module || true
	sudo rmmod rotary_module || true
	dmesg | tail -n 20

reload: unload load
