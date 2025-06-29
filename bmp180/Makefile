obj-m := bmp180.o

# 커널 소스 디렉토리
KDIR := $(HOME)/project/linux

# 현재 디렉토리
PWD := $(shell pwd)

# 크로스 컴파일러 설정
CROSS_COMPILE := aarch64-linux-gnu-
ARCH := arm64

all:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules -j12

clean:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean