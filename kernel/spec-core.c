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
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <asm/unaligned.h>

#include "spec.h"
#include "loader-ll.h"

static char *spec_name = "%b";
module_param_named(name, spec_name, charp, 0444);

/*
 * A procedure to build the names associated with the device.  This
 * copies the spec_name. With "spec-" prefix, expanding %P
 * (vendor:device), %p (subv:subd), %b (bus:devfn).  Then ".bin" is
 * the gateware, "-lm32.bin" is the lm32 compiled code and ".ko"
 * is the kernel module.
 * Example for device in bus 2, slot 0:
 * command: "insmod spec name=%b"
 * gateware: "spec-B0002.bin"
 * program: "spec-B0002-cpu.bin" -- will be .elf, hopefully
 * other module requested: "spec-B0002.ko"
 */
static int spec_build_names(struct spec_dev *dev)
{
	struct pci_dev *pdev = dev->pdev;
	char basename[64];
	char *si, *so;
	int i;
	static char *templates[] = {
		[SPEC_NAME_FW] = "spec-%s.bin",
		[SPEC_NAME_PROG] = "spec-%s-cpu.bin", /* will be .elf */
		[SPEC_NAME_SUBMOD] = "spec-%s", /* .ko added by modprobe */
	};

	for (si = spec_name, so = basename; *si ; si++) {
		if (so - basename >= sizeof(basename))
			return -ENOSPC;
		if (*si != '%') {
			*so++ = *si;
			continue;
		}
		si++; /* eat '%' */
		if (so - basename + 5 >= sizeof(basename))
			return -ENOSPC;
		switch(*si) {
		case 'b': /* BUS id */
			so += sprintf(so, "B%04x", pdev->bus->number);
			break;
		case 's': /* slot-fn id */
			so += sprintf(so, "S%04x", pdev->devfn);
			break;
		case '%':
			*so++ = '%';
		default:
			return -EINVAL;
		}
	}
	/* terminate and remove trailing spaces (includes newlines) */
	*so = '\0';
	while (isspace(*--so))
		*so = '\0';

	/* build the actual things */
	for (i = 0; i < SPEC_NAMES; i++)
		dev->names[i] = kasprintf(GFP_KERNEL, templates[i], basename);
	return 0;
}

/* Load the FPGA. This bases on loader-ll.c, a kernel/user space thing */
static int spec_load_fpga(struct spec_dev *dev)
{
	const struct firmware *fw;
	unsigned long j;
	int i, err, wrote, done;

	err = request_firmware(&fw, dev->names[SPEC_NAME_FW], &dev->pdev->dev);
	if (err < 0)
		return err;
	pr_info("%s: got binary file \"%s\", %i (0x%x) bytes\n", __func__,
		dev->names[SPEC_NAME_FW], fw->size, fw->size);

	/* loader_low_level is designed to run from user space too */
	wrote = loader_low_level(0 /* unused fd */,
				 dev->remap[2], fw->data, fw->size);
	j = jiffies + 2 * HZ;
	/* Wait for DONE interrupt  */
	while(!done) {
		i = readl(dev->remap[2] + FCL_IRQ);
		if (i & 0x8) {
			printk("%s: done after %i writes\n", __func__,
			       wrote);
			done = 1;
		} else if( (i & 0x4) && !done) {
			printk("%s: error after %i writes\n", __func__,
			       wrote);
			err = -ETIMEDOUT;
			goto out;
		}

		if (time_after(jiffies, j)) {
			printk("%s: timeout after %i writes\n", __func__,
			       wrote);
			err = -ETIMEDOUT;
			goto out;
		}
	}
out:
	release_firmware(fw);
        return err;
}

static int spec_load_submodule(struct spec_dev *dev)
{
	int err;

	err = request_module(dev->names[SPEC_NAME_SUBMOD]);
	pr_info("%s: load \"%s\": %i\n", __func__,
		dev->names[SPEC_NAME_SUBMOD], err);
	return err;
}

int spec_load_lm32(struct spec_dev *dev)
{
	const struct firmware *fw;
	int err, off;

	err = request_firmware(&fw, dev->names[SPEC_NAME_PROG],
			       &dev->pdev->dev);
	if (err < 0)
		return err;
	pr_info("%s: got program file \"%s\", %i (0x%x) bytes\n", __func__,
		dev->names[SPEC_NAME_PROG], fw->size, fw->size);

	/* Reset the LM32 */
	writel(0x1deadbee, dev->remap[0] + 0xA0400);

	/* Copy stuff over */
	for (off = 0; off < fw->size; off += 4) {
		uint32_t datum;

		datum = get_unaligned_be32(fw->data + off);
		writel(datum, dev->remap[0] + 0x80000 + off);
	}
	/* Unreset the LM32 */
	writel(0xdeadbee, dev->remap[0] + 0xA0400);

	/* MSC */
	pr_info("LM32 has been restarted\n");
	release_firmware(fw);
	return 0;
}


/* A procedure to load the three names */
static int spec_load_files(struct spec_dev *dev)
{
	int err;

	printk("%s\n", __func__);

	/*
	 * We need to load the three files and we are in process context
	 * (god has said: Documentation/PCI/pci.txt)
	 */
	if ( (err = spec_load_fpga(dev)) < 0) {
		dev_err(&dev->pdev->dev, "Can't load firwmare \"%s\" - %i\n",
			dev->names[SPEC_NAME_FW], err);
		return err;
	}

	if ( (err = spec_load_lm32(dev)) < 0 ) {
		dev_warn(&dev->pdev->dev, "Can't load program \"%s\" - %i\n",
			dev->names[SPEC_NAME_PROG], err);
		/* continue anyways */
	}
	if ( (err = spec_load_submodule(dev)) < 0 )
		dev_warn(&dev->pdev->dev, "Can't load submodule \"%s\" - %i\n",
			dev->names[SPEC_NAME_SUBMOD], err);
	return 0;
}

struct list_head spec_list;
EXPORT_SYMBOL(spec_list);

/* The probe can be called in atomic context, so all loading is delayed */
static int spec_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct spec_dev *dev;
	int i;

	pr_error(KBUILD_MODNAME "spec device probed\n");
	printk("%s (device %04x:%04x)\n", __func__, pdev->bus->number,
		pdev->devfn);
	printk("%s: current %i (%s)\n", __func__, current->pid, current->comm);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->pdev = pdev;

	/* Remap our 3 bars */
	for (i = 0; i < 3; i++) {
		struct resource *r = pdev->resource + (2 * i);
		if (!r->start)
			continue;
		dev->area[i] = r;
		if (r->flags & IORESOURCE_MEM)
			dev->remap[i] = ioremap(r->start,
						r->end + 1 - r->start);
	}

	/* Build the names */
	if (spec_build_names(dev) < 0) {
		dev_warn(&pdev->dev, "can't build names with \"%s\"\n",
			 spec_name);
		/* go on anyways */
	}

	/* Register the device in out list, so the submodule will find it */
	pci_set_drvdata(pdev, dev);
	list_add(&dev->list, &spec_list);

	/* The probe function can sleep, so load firmware directly */
	spec_load_files(dev);

	/* Done */
	return 0;
}

static void spec_remove(struct pci_dev *pdev)
{
	struct spec_dev *dev = pci_get_drvdata(pdev);
	int i;

	printk("%s\n", __func__);

	for (i = 0; i < 3; i++) {
		iounmap(dev->remap[i]);
		dev->remap[i] = NULL;
		dev->area[i] = NULL;
	}
	list_del(&dev->list);
	pci_set_drvdata(pdev, NULL);
	for (i = 0; i < SPEC_NAMES; i++)
		kfree(dev->names[i]);
	kfree(dev);
}


DEFINE_PCI_DEVICE_TABLE(spec_idtable) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CERN, PCI_DEVICE_ID_SPEC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_GENNUM, PCI_DEVICE_ID_GN4124) },
	{ 0,},
};

static struct pci_driver spec_driver = {
	.name = "spec",
	.id_table = spec_idtable,
	.probe = spec_probe,
	.remove = spec_remove,
};

static int spec_init(void)
{
	INIT_LIST_HEAD(&spec_list);
	return pci_register_driver(&spec_driver);
}

static void spec_exit(void)
{
	pci_unregister_driver(&spec_driver);

}

module_init(spec_init);
module_exit(spec_exit);

MODULE_LICENSE("GPL");
