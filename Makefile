obj-m := nrf24.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
O=sun4i
ARCH=arm
CROSS_COMPILE=arm-linux-gnu-
all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
