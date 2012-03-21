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
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <asm/unaligned.h>

#include "spec.h"

#include "wr-nic.h"

/*
 * nic-device.c defines a platform driver. We need a platform device
 * for each pci board -- code from wr_nic/module.c
 */

/* The WRN_RES_ names are defined in the header file. Each block 64kB */
#define __RES(name_) {						\
	.start = FPGA_BASE_ ## name_,				\
	.end =   FPGA_BASE_ ## name_ + FPGA_SIZE_ ## name_ - 1,	\
	.flags = IORESOURCE_MEM				\
	}

/* Not all the blocks are relevant to this driver, only list the used ones */
static struct resource wrn_resources[] = {
	[WRN_FB_NIC] = __RES( NIC ),
	[WRN_FB_EP] = __RES( EP ),
	[WRN_FB_TS] = __RES( TS ),
	[WRN_FB_PPSG] = __RES( PPSG ), /* WARNING: We _know_ it's 4!!!! */
};
#undef __RES

static void wrn_release(struct device *dev)
{
	/* nothing to do, but mandatory function */
	pr_debug("%s\n", __func__);
}

static struct platform_device wrn_device = {
	.name = DRV_NAME,
	.id = 0,
	.resource = wrn_resources,
	.num_resources = 0, // -- can't be resources... ARRAY_SIZE()
	.dev = {
		.platform_data = 0, /* to be filled */
		.release = &wrn_release,
		/* dma_mask not used, as we make no DMA */
	},
};





/* Interrupt handler, currently doint nothing */
irqreturn_t wrn_irq(int irq, void *dev_id)
{
	struct spec_dev *dev = dev_id;
	static unsigned long j;

	/* ack the irq */
	readl(dev->remap[2] + 0xa20 /* FIXME: use spec.h names */);
	dev->irqcount++;

	/* Print, but no more than 10 times per second */
	if (j == 0 || time_after(jiffies, j + HZ / 10)) {
		dev_info(&dev->pdev->dev, "irq %i (count %li)\n", irq,
			 dev->irqcount);
		j = jiffies;
	}
	return IRQ_HANDLED;
}


/* This should register a network device */
static int wrn_probe(struct spec_dev *dev)
{
	int err;
	uint32_t val;
	struct wrn_dev *wrndev;

	err = request_irq(dev->pdev->irq, wrn_irq, IRQF_SHARED, "wr-nic", dev);
	if (err < 0) {
		dev_err(&dev->pdev->dev, "can't request irq %i (err %i)\n",
			dev->pdev->irq, err);
		return err;
	}

	/* GENNUM MAGIC: Enable mutiple-msi and gpio irq in the proper reg */
	writel(0xa55805, dev->remap[2] + 0x48);
	val = readl(dev->remap[2] + 0x54);
	//writel(0x8000, dev->remap[2] + GN_INT_CFG0 + 4 * (val & 3));

	/* Make a copy of the device and register it -- FIXME: error check */
	dev->platdev = kmemdup(&wrn_device, sizeof(wrn_device), GFP_KERNEL);
	wrndev = kzalloc(sizeof(struct wrn_dev), GFP_KERNEL);
	wrndev->spec = dev;
	dev->platdev->dev.platform_data = wrndev;
	platform_device_register(dev->platdev);
	return 0;
}

static void wrn_remove(struct spec_dev *dev)
{
	free_irq(dev->pdev->irq, dev);

	/* deallocate what has been allocate */
	platform_device_unregister(dev->platdev);
	kfree(dev->platdev->dev.platform_data);
	kfree(dev->platdev);
	dev->platdev = NULL;
}

static int wrn_init(void)
{
	struct spec_dev *dev;

	platform_driver_register(&wrn_driver); /* nic-device.c */

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

	platform_driver_unregister(&wrn_driver); /* nic-device.c */

}

module_init(wrn_init);
module_exit(wrn_exit);

MODULE_LICENSE("GPL");

