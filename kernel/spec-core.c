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
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/pci.h>
#include <linux/io.h>

#include "spec.h"

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
 * gateware: "spec-0002:0000.bin"
 * program: "spec-0002:0000-cpu.bin" -- will be .elf, hopefully
 * other module requested: "spec-0002:0000.ko"
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
		[SPEC_NAME_SUBMOD] = "spec-%s.ko",
	};

	for (si = spec_name, so = basename; *si ; si++) {
		if (so - basename >= sizeof(basename))
			return -ENOSPC;
		if (*si != '%') {
			*so++ = *si;
			continue;
		}
		si++; /* eat '%' */
		if (so - basename + 9 >= sizeof(basename))
			return -ENOSPC;
		switch(*si) {
		case 'P': /* PCI vendor:device */;
			so += sprintf(so, "%04x:%04x",
				pdev->vendor, pdev->device);
			break;
		case 'p': /* PCI subvendor:subdevice */;
			so += sprintf(so, "%04x:%04x",
				pdev->subsystem_vendor, pdev->subsystem_device);
			break;
		case 'b': /* BUS id */
			so += sprintf(so, "%04x:%04x",
				pdev->bus->number, pdev->devfn);
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
	printk("%s: basename is \"%s\"\n", __func__, basename);

	/* build the actual things */
	for (i = 0; i < SPEC_NAMES; i++)
		dev->names[i] = kasprintf(GFP_KERNEL, templates[i], basename);
	return 0;
}

/* A procedure to load the three names */
static int spec_load_files(struct spec_dev *dev)
{
	printk("%s: not implemented\n", __func__);
	return 0;
}

static struct list_head spec_list;

/* The probe can be called in atomic context, so all loading is delayed */
static int spec_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct spec_dev *dev;
	int i;

	printk("%s\n", __func__);
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

	/* The probe function can sleep, so load firmware directly */
	spec_load_files(dev);

	/* Done */
	pci_set_drvdata(pdev, dev);
	list_add(&dev->list, &spec_list);
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

	/* FIXME */
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
