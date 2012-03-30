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
#include <linux/cdev.h>
#include <linux/fs.h>

#include "spec.h"

#define FMC_ADC_MAX_DEVICES 32

struct class *fadc_class;
static dev_t fadc_devno;

struct fadc_dev {
	int ndev;
	struct spec_dev *spec;
	struct cdev cdev;
	struct device *dev;
	struct module *owner;
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

/* Device operations */

static int fadc_device_open(struct inode *inode, struct file *file)
{
	struct fadc_dev *fadcdev;

	fadcdev = container_of(inode->i_cdev, struct fadc_dev, cdev);
	if (!try_module_get(fadcdev->owner))
		return -ENODEV;
	file->private_data = fadcdev;

	return 0;
}

static int fadc_device_release(struct inode *inode, struct file *file)
{
	struct fadc_dev *fadcdev;

	/* DOUBT: can file->private_data ever end up as NULL? */
	fadcdev = file->private_data;
	module_put(fadcdev->owner);

	return 0;
}

static ssize_t fadc_device_read(struct file *f, char *buf, size_t count,
		loff_t *ppos)
{
	struct fadc_dev *fadcdev;

	fadcdev = f->private_data;

	return 0;
}

static ssize_t fadc_device_write(struct file *f, const char *buf, size_t count,
		loff_t *ppos)
{
	struct fadc_dev *fadcdev;

	if (count == 0)
		return 0;

	fadcdev = f->private_data;

	return 0;
}

struct file_operations fadc_fops = {
	.owner = THIS_MODULE,
	.open = fadc_device_open,
	.release = fadc_device_release,
	.read = fadc_device_read,
	.write = fadc_device_write,
};


static int __devinit fadc_probe(struct platform_device *pdev)
{
	int ret;
	dev_t devno;
	static int ndev;
	struct fadc_dev *fadcdev;

	fadcdev = pdev->dev.platform_data;

	ndev = fadcdev->ndev;
	devno = MKDEV(MAJOR(fadc_devno), ndev);
	cdev_init(&fadcdev->cdev, &fadc_fops);
	fadcdev->cdev.owner = THIS_MODULE;
	fadcdev->cdev.ops = &fadc_fops;
	printk("cdev_add\n");
	ret = cdev_add(&fadcdev->cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "error %d adding cdev %d\n", ret, ndev);
		goto cdev_add_fail; 
	}

	printk("device_create\n");
	fadcdev->dev = device_create(fadc_class, &pdev->dev, devno, fadcdev, "spec_adc%i", ndev);
	if (IS_ERR(fadcdev->dev)) {
		ret = PTR_ERR(fadcdev->dev);
		printk(KERN_ERR "error %d creating device %d\n", ret, ndev);
		goto device_create_fail;
	}

	return 0;

device_create_fail:
	cdev_del(&fadcdev->cdev);
cdev_add_fail:
	return ret;
}

static int fadc_remove(struct platform_device *pdev)
{
	struct fadc_dev *fadcdev;

	fadcdev = pdev->dev.platform_data;

	printk("device_destroy\n");
	device_destroy(fadc_class, fadcdev->ndev);
	printk("cdev_del\n");
	cdev_del(&fadcdev->cdev);

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
	fadcdev->owner = THIS_MODULE;
	fadcdev->ndev = fadc_plat_device.id;
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
        int ret;
	struct spec_dev *dev;

	/* allocate the character major/minor region */
	ret = alloc_chrdev_region(&fadc_devno, 0, FMC_ADC_MAX_DEVICES, "fmcadc");
	if (ret) {
		printk(KERN_ERR "alloc_chrdev_region failed\n");
		goto alloc_chr_fail;
	}

	/* create a sysfs class */
	fadc_class = class_create(THIS_MODULE, "fmcadc");
	if (IS_ERR(fadc_class)) {
		printk(KERN_ERR "class_create failed\n");
		ret = PTR_ERR(fadc_class);
		goto class_create_fail;
	}

	platform_driver_register(&fadc_driver);

	/* Scan the list and see what is there. Take hold of everything */
	list_for_each_entry(dev, &spec_list, list) {
		printk("%s: init %04x:%04x (%pR - %p)\n", __func__,
				dev->pdev->vendor, dev->pdev->device,
				dev->area[0], dev->remap[0]);
		fadc_init_probe(dev);
	}
	return 0;

class_create_fail:
	unregister_chrdev_region(fadc_devno, FMC_ADC_MAX_DEVICES);
alloc_chr_fail:
	return ret;
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
	class_destroy(fadc_class);
	unregister_chrdev_region(fadc_devno, FMC_ADC_MAX_DEVICES);
}

module_init(fadc_init);
module_exit(fadc_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC+ADC Linux Driver");
MODULE_LICENSE("GPL");
