#include <linux/init.h>
#include <linux/module.h>

static int __init spec_adc_init(void)
{
	return 0;
}

static void __exit spec_adc_exit(void)
{
}

module_init(spec_adc_init);
module_exit(spec_adc_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC+ADC Linux Driver");
MODULE_LICENSE("GPL");
