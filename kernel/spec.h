/*
 * Copyright (C) 2010-2012 CERN (www.cern.ch)
 * Author: Alessandro Rubini <rubini@gnudd.com>
 *
 * Released according to the GNU GPL, version 2 or any later version.
 *
 * This work is part of the White Rabbit project, a research effort led
 * by CERN, the European Institute for Nuclear Research.
 */
#ifndef __SPEC_H__
#define __SPEC_H__
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/firmware.h>
#include <linux/atomic.h>
#include <linux/list.h>

/* We identify both newer SPECs with their own ID and older ones */
#define PCI_VENDOR_ID_CERN	0x10dc
#define PCI_DEVICE_ID_SPEC		0x018d
#define PCI_VENDOR_ID_GENNUM	0x1a39
#define PCI_DEVICE_ID_GN4124		0x0004

enum spec_names {
	SPEC_NAME_FW,
	SPEC_NAME_PROG,
	SPEC_NAME_SUBMOD,
	SPEC_NR_NAMES,
};

#define SPEC_DEFAULT_FW_NAME "spec_top.bin"

/* Our device structure */
struct spec_dev {
	struct pci_dev		*pdev;
	struct resource		*area[3];	/* bar 0, 2, 4 */
	void			*remap[3];	/* ioremap of bar 0, 2, 4 */
	char			*names[SPEC_NR_NAMES];
	char			*submod_name;
	spinlock_t		lock;
	struct work_struct	work;
	const struct firmware	*fw;
	struct list_head	list;
	unsigned long		irqcount;
	atomic_t		has_submod;
};

/* Used by sub-modules */
extern struct list_head spec_list;

/* Registers from the gennum header files */
enum {
	GNGPIO_BASE = 0xA00,
	GNGPIO_DIRECTION_MODE = GNGPIO_BASE + 0x4,
	GNGPIO_OUTPUT_ENABLE = GNGPIO_BASE + 0x8,
	GNGPIO_OUTPUT_VALUE = GNGPIO_BASE + 0xC,
	GNGPIO_INPUT_VALUE = GNGPIO_BASE + 0x10,

	FCL_BASE	= 0xB00,
	FCL_CTRL	= FCL_BASE,
	FCL_STATUS	= FCL_BASE + 0x4,
	FCL_IODATA_IN	= FCL_BASE + 0x8,
	FCL_IODATA_OUT	= FCL_BASE + 0xC,
	FCL_EN		= FCL_BASE + 0x10,
	FCL_TIMER_0	= FCL_BASE + 0x14,
	FCL_TIMER_1	= FCL_BASE + 0x18,
	FCL_CLK_DIV	= FCL_BASE + 0x1C,
	FCL_IRQ		= FCL_BASE + 0x20,
	FCL_TIMER_CTRL	= FCL_BASE + 0x24,
	FCL_IM		= FCL_BASE + 0x28,
	FCL_TIMER2_0	= FCL_BASE + 0x2C,
	FCL_TIMER2_1	= FCL_BASE + 0x30,
	FCL_DBG_STS	= FCL_BASE + 0x34,
	FCL_FIFO	= 0xE00,
	PCI_SYS_CFG_SYSTEM = 0x800
};

#endif /* __SPEC_H__ */
