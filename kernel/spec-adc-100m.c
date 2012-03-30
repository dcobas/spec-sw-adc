#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <asm/unaligned.h>

#include "spec.h"

struct fadc_dev {
	struct spec_dev *spec;
};

/* Interrupt handler, currently doint nothing */
irqreturn_t fadc_irq(int irq, void *dev_id)
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

static int __devinit fadc_probe(struct platform_device *pdev)
{
	return 0;
}

static int fadc_remove(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver fadc_driver = {
	.probe = fadc_probe,
	.remove = fadc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
	},
};

static void fadc_release(struct device *dev)
{
	/* nothing to do, but mandatory function */
	pr_debug("%s\n", __func__);
}

static struct platform_device fadc_plat_device = {
	.name = KBUILD_MODNAME,
	.id = 0,
	.resource = 0/*fadc_resources*/,
	.num_resources = 0, // -- can't be resources... ARRAY_SIZE()
	.dev = {
		.platform_data = 0, /* to be filled */
		.release = &fadc_release,
		/* dma_mask not used, as we make no DMA */
	},
};

static int fadc_init_probe(struct spec_dev *dev)
{
	int err;
	uint32_t val;
	struct fadc_dev *fadcdev;

	printk("spec_adc_probe\n");
	err = request_irq(dev->pdev->irq, fadc_irq, IRQF_SHARED, "wr-nic", dev);
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
	dev->platdev = kmemdup(&fadc_plat_device, sizeof(fadc_plat_device), GFP_KERNEL);
	fadcdev = kzalloc(sizeof(struct fadc_dev), GFP_KERNEL);
	fadcdev->spec = dev;
	dev->platdev->dev.platform_data = fadcdev;
	platform_device_register(dev->platdev);

	/* increment ID for next time */
	fadc_plat_device.id++;

	return 0;
}

static void fadc_init_remove(struct spec_dev *dev)
{
	free_irq(dev->pdev->irq, dev);

	/* deallocate what has been allocate */
	platform_device_unregister(dev->platdev);
	kfree(dev->platdev->dev.platform_data);
	kfree(dev->platdev);
	dev->platdev = NULL;
}

static int __init fadc_init(void)
{
	struct spec_dev *dev;

	platform_driver_register(&fadc_driver);

	/* Scan the list and see what is there. Take hold of everything */
	list_for_each_entry(dev, &spec_list, list) {
		printk("%s: init %04x:%04x (%pR - %p)\n", __func__,
				dev->pdev->vendor, dev->pdev->device,
				dev->area[0], dev->remap[0]);
		fadc_init_probe(dev);
	}
	return 0;
}

static void __exit fadc_exit(void)
{
	struct spec_dev *dev;

	list_for_each_entry(dev, &spec_list, list) {
		printk("%s: release %04x:%04x (%pR - %p)\n", __func__,
				dev->pdev->vendor, dev->pdev->device,
				dev->area[0], dev->remap[0]);
		fadc_init_remove(dev);
	}

	platform_driver_unregister(&fadc_driver);
}

module_init(fadc_init);
module_exit(fadc_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC+ADC Linux Driver");
MODULE_LICENSE("GPL");
