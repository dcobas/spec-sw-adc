#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ctype.h>

#define SPEC_ADC_MAX_DEVICES		32

static short buses[SPEC_ADC_MAX_DEVICES];
static unsigned int bus_num;
module_param_array(buses, long, &bus_num, 0444);
MODULE_PARM_DESC(buses, "The buses to use for SPEC+ADC");

static short devfns[SPEC_ADC_MAX_DEVICES];
static unsigned int devfn_num;
module_param_array(devfns, long, &devfn_num, 0444);
MODULE_PARM_DESC(devfns, "The device functions to use for SPEC+ADC");

static char *sadc_name = "%b";
module_param_named(name, sadc_name, charp, 0444);
MODULE_PARM_DESC(name, "The device functions to use for SPEC+ADC");

char *list[SPEC_ADC_MAX_DEVICES];

#if 0
static int sadc_probe(struct pci_dev *pdev)
{
	printk("Hello ADC world!\n");
	return 0;
}

static void sadc_remove(struct pci_dev *pdev)
{
	printk("Bye ADC world!\n");
}
#endif

static int sadc_build_names(void)
{
	char basename[64];
	char *si, *so;
	int i;

	for (i = 0; i < bus_num; i++) {
		for (si = sadc_name, so = basename; *si ; si++) {
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
				so += sprintf(so, "B%04x", buses[i]);
				break;
			case 's': /* slot-fn id */
				so += sprintf(so, "S%04x", devfns[i]);
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
		list[i] = kasprintf(GFP_KERNEL, "spec-%s", basename);
	}
	return 0;
}


static int __init sadc_init(void)
{
	int i;

	sadc_build_names();
	for (i = 0; i < bus_num; i++) {
		printk("sadc_name: %s\n", list[i]);
	}
	return 0;
}

static void __exit sadc_exit(void)
{
	int i;

	for (i = 0; i < bus_num; i++)
		kfree(list[i]);
}

module_init(sadc_init);
module_exit(sadc_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC+ADC Linux Driver");
MODULE_LICENSE("GPL");
