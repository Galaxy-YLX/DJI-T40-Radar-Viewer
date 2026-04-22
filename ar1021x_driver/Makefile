KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)/ath6kl modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)/ath6kl clean

install: all
	@mkdir -p /lib/modules/$(shell uname -r)/updates
	cp $(PWD)/ath6kl/ath6kl_core.ko /lib/modules/$(shell uname -r)/updates/
	cp $(PWD)/ath6kl/ath6kl_usb.ko /lib/modules/$(shell uname -r)/updates/
	depmod -a

load:
	-rmmod ath6kl_usb ath6kl_core 2>/dev/null
	modprobe cfg80211
	insmod $(PWD)/ath6kl/ath6kl_core.ko
	insmod $(PWD)/ath6kl/ath6kl_usb.ko

unload:
	-rmmod ath6kl_usb ath6kl_core 2>/dev/null

.PHONY: all clean install load unload
