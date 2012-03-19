/*
 * Copyright (C) 2010-2012 CERN (www.cern.ch)
 * Author: Alessandro Rubini <rubini@gnudd.com>
 *
 * Released according to the GNU GPL, version 2 or any later version.
 *
 * This work is part of the White Rabbit project, a research effort led
 * by CERN, the European Institute for Nuclear Research.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <asm/unaligned.h>

#include "spec.h"

#include "wr-nic.h"

irqreturn_t wrn_irq(int irq, void *dev_id)
{
	printk("%s:%i\n", __func__, irq);
	return IRQ_NONE;
}


/* This should register a network device */
static int wrn_probe(struct spec_dev *dev)
{
	int i;

	for (i = dev->pdev->irq; i < dev->pdev->irq + 1; i++) {
		if (request_irq(i, wrn_irq, 0,
				"wr-nic", dev) < 0) {
			pr_err("%s: can't request irq %i\n", __func__, i);
		}
	}
	return 0;
}

static void wrn_remove(struct spec_dev *dev)
{
	int i;

	for (i = dev->pdev->irq; i < dev->pdev->irq + 1; i++) {
		free_irq(i, dev);
	}
}

static int wrn_init(void)
{
	struct spec_dev *dev;

	/* Scan the list and see what is there. Take hold of everything */
	list_for_each_entry(dev, &spec_list, list) {
		printk("%s: init %04x:%04x (%pR - %p)\n", __func__,
		       dev->pdev->vendor, dev->pdev->device,
		       dev->area[0], dev->remap[0]);
		wrn_probe(dev);
	}
	return 0;
}

static void wrn_exit(void)
{
	struct spec_dev *dev;

	list_for_each_entry(dev, &spec_list, list) {
		printk("%s: release %04x:%04x (%pR - %p)\n", __func__,
		       dev->pdev->vendor, dev->pdev->device,
		       dev->area[0], dev->remap[0]);
		wrn_remove(dev);
	}

}

module_init(wrn_init);
module_exit(wrn_exit);

MODULE_LICENSE("GPL");

