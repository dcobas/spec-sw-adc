
LINUX ?= /lib/modules/$(shell uname -r)/build

obj-m = spec.o
obj-m += spec-wr-nic.o
obj-m += spec-fine-delay.o

spec-objs = spec-core.o loader-ll.o

spec-wr-nic-objs = wr-nic.o nic-core.o nic-device.o

all modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules

install modules_install:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules_install


clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
