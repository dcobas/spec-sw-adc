
LINUX ?= /lib/modules/$(shell uname -r)/build

obj-m = spec.o
obj-m += spec-wr-nic.o
obj-m += spec-fine-delay.o
obj-m += fmcadc.o

spec-objs = spec-core.o loader-ll.o

all modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules

install modules_install:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules_install


clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order

load:
	insmod ./spec.ko name=%b%s
	insmod ./fmcadc.ko

unload:
	rmmod fmcadc
	rmmod spec
