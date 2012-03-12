
CROSS_COMPILE ?= /opt/gcc-lm32/bin/lm32-elf-

# classic cross-compilation tool-set
AS		= $(CROSS_COMPILE)as
LD		= $(CROSS_COMPILE)ld
CC		= $(CROSS_COMPILE)gcc
CPP		= $(CC) -E
AR		= $(CROSS_COMPILE)ar
NM		= $(CROSS_COMPILE)nm
STRIP		= $(CROSS_COMPILE)strip
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump


CFLAGS += -ffreestanding -Os  \
	-ffunction-sections -fdata-sections \
	-mmultiply-enabled -mbarrel-shift-enabled

LDFLAGS = -nostdlib -static -T spec.lds

ALL = $(patsubst %.c, %.bin, $(wildcard *.c))

%.bin: %
	$(OBJCOPY) -O binary $* $*.bin

all: $(ALL)

clean:
	rm -f $(ALL) *.o *~

spec-test00-cpu: crt0.o spec-test00-cpu.o
	$(CC) $(LDFLAGS) $^ -o $@
