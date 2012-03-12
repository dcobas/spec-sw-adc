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
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <asm/unaligned.h>

#include "spec.h"

static int wrn_init(void)
{
}

static void wrn_exit(void)
{

}

module_init(wrn_init);
module_exit(wrn_exit);

MODULE_LICENSE("GPL");

