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
#include <linux/init.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/io.h>

#include "spec.h"

static char *spec_names[SPEC_MAX_BOARDS];
static int spec_n_names;

module_param_array_named(name, spec_names, charp, &spec_n_names, 0444);


DEFINE_PCI_DEVICE_TABLE(spec_idtable) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CERN, PCI_DEVICE_ID_SPEC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_GENNUM, PCI_DEVICE_ID_GN4124) },
	{ 0,},
};

/* The loader is laid out as a kernel thread, to simplify stuff */

/* FIXME */

static struct list_head spec_list;

/* The probe can be called in atomic context, so all loading is delayed */
static int spec_probe (struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct spec_dev *dev;
	printk("%s\n", __func__);
	/* FIXME */
	return 0;
}

static void spec_remove(struct pci_dev *pdev)
{
	printk("%s\n", __func__);
	/* FIXME */
}

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
