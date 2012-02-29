
LINUX ?= /lib/modules/$(shell uname -r)/build

obj-m = spec.o
obj-m += spec-fine-delay.o

spec-objs = spec-core.o loader-ll.o

all modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules

clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
