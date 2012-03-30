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

#include "fmcadc.h"
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

#define FMC_ADC_CHANNEL_1       0
#define FMC_ADC_CHANNEL_2       1
#define FMC_ADC_CHANNEL_3       2
#define FMC_ADC_CHANNEL_4       3

#define FADC_CSR(dev, addr) \
	(dev->spec->remap[0] + FADC_FMC_CSR_ADDR + addr)

unsigned int fmc_adc_channel_addr(int channel, int reg)
{
	return (reg + 0x10*channel);
}

void fmc_adc_set_gain(struct fadc_dev *dev, int channel, int value)
{
	int ch_addr;

	ch_addr = FADC_FMC_CSR_ADDR + 0x34 + 0x10*channel;
	writel(value, dev->spec->remap[0] + ch_addr);
}

void fadc_trig_led(struct fadc_dev *dev, int state)
{
	int reg;

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	if (state)
		reg |= (1 << FADC_CTL_TRIG_LED);
	else
		reg &= ~(1 << FADC_CTL_TRIG_LED);
	reg &= FADC_CTL_MASK;
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
	reg = readl(FADC_CSR(dev, FADC_R_CTL));
}

void fadc_acq_led(struct fadc_dev *dev, int state)
{
	int reg;

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	if (state)
		reg |= (1 << FADC_CTL_ACQ_LED);
	else
		reg &= ~(1 << FADC_CTL_ACQ_LED);
	reg &= FADC_CTL_MASK;
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
	reg = readl(FADC_CSR(dev, FADC_R_CTL));
}

void fmc_adc_set_input_range(struct fadc_dev *dev, int channel, int in_range)
{
	//Set input range
	unsigned int ssr;
	unsigned int reg;
	unsigned int addr;

	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_SSR);
	ssr = readl(FADC_CSR(dev, addr));
	reg = FADC_IN_TERM_MASK & ssr;
	reg |= in_range;
	writel(reg, FADC_CSR(dev, addr));
}

/* Set SSR register */
void fmc_adc_set_ssr(struct fadc_dev *dev, int ch, int val)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(ch, FADC_R_CH1_SSR);
	writel(val, FADC_CSR(dev, addr));
}

/* Get SSR register */
unsigned int fmc_adc_get_ssr(struct fadc_dev *dev, int ch)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(ch, FADC_R_CH1_SSR);
	return readl(FADC_CSR(dev, addr));
}

/* Set trigger configuration */
void fmc_adc_set_trig_config(struct fadc_dev *dev, unsigned int hw_sel,
	unsigned int ext_pol, unsigned int hw_en, unsigned int sw_en,
	unsigned int int_sel, unsigned int int_thres, unsigned int delay)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	/* Hardware trigger select (ext/int) */
	if (hw_sel)
		reg |= (1 << FADC_TRIG_CFG_HW_SEL);
	/* External trigger pulse polarity */
	if (ext_pol)
		reg |= (1 << FADC_TRIG_CFG_EXT_POL);
	/* Hardware trigger enable */
	if (hw_en)
		reg |= (1 << FADC_TRIG_CFG_HW_EN);
	/* Software trigger enable */
	if (sw_en)
		reg |= (1 << FADC_TRIG_CFG_SW_EN);
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
	/* Internal trigger channel select (1 to 4) */
	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	reg &= FADC_INT_SEL_MASK;
	reg += (int_sel << FADC_TRIG_CFG_INT_SEL);
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
	/* Internal trigger threshold */
	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	reg &= FADC_INT_THRES_MASK;
	reg += (int_thres << FADC_TRIG_CFG_INT_THRES);
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
	/* Trigger delay (in sampling clock ticks) */
	writel(delay, FADC_CSR(dev, FADC_R_TRIG_DLY));
}

/*  Get trigger configuration */

/*  Internal trigger */
void fmc_adc_set_int_trig(struct fadc_dev *dev, int channel, int polarity,
	int threshold, int int_sel)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	/*  Software trigger disable */
	reg &= ~FADC_TRIG_CFG_SW_EN;
	/*  Select internal hardware trigger */
	reg &= ~FADC_TRIG_CFG_HW_SEL;
	/*  Trigger polarity */
	if (polarity)
		reg |= FADC_TRIG_CFG_EXT_POL;
	else
		reg &= ~FADC_TRIG_CFG_EXT_POL;
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
	/*  Internal trigger channel select (1 to 4) */
	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	reg &= FADC_INT_SEL_MASK;
	reg += (int_sel << FADC_TRIG_CFG_INT_SEL);
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
	/*  Internal trigger threshold */
	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	reg &= FADC_INT_THRES_MASK;
	reg += (threshold << FADC_TRIG_CFG_INT_THRES);
	/*  Hardware trigger enable */
	reg |= FADC_TRIG_CFG_HW_EN;
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
}

/*  External trigger */
void fmc_adc_set_ext_trig(struct fadc_dev *dev, int polarity)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	reg &= ~FADC_TRIG_CFG_SW_EN;
	reg |= FADC_TRIG_CFG_HW_SEL;
	if (polarity)
		reg |= FADC_TRIG_CFG_EXT_POL;
	else
		reg &= ~FADC_TRIG_CFG_EXT_POL;
	reg |= FADC_TRIG_CFG_HW_EN;
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
}

/*  Software trigger */
void fmc_adc_set_soft_trig(struct fadc_dev *dev)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_TRIG_CFG));
	/*  Hardware trigger disable */
	reg &= ~FADC_TRIG_CFG_HW_EN;
	/*  Software trigger enable */
	reg |= FADC_TRIG_CFG_SW_EN;
	writel(reg, FADC_CSR(dev, FADC_R_TRIG_CFG));
}

/*  Trigger delay */
void fmc_adc_set_trig_delay(struct fadc_dev *dev, int delay)
{
	/*  Trigger delay (in sampling clock ticks) */
	writel(delay, FADC_CSR(dev, FADC_R_TRIG_DLY));
}

/*  Enable test data */
void fmc_adc_test_data_en(struct fadc_dev *dev)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	reg |= ((1 << FADC_CTL_TEST_DATA_EN) & FADC_CTL_MASK);
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
}

/*  Disable test data */
void fmc_adc_test_data_dis(struct fadc_dev *dev)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	reg &= (~(1 << FADC_CTL_TEST_DATA_EN) & FADC_CTL_MASK);
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
}

/*  Get serdes sync status */
int fmc_adc_get_serdes_sync_stat(struct fadc_dev *dev)
{
	return ((readl(FADC_CSR(dev, FADC_R_STA)) & FADC_STA_SERDES_SYNCED) ? 1 : 0);
}

/*  Start acquisition */
void fmc_adc_start_acq(struct fadc_dev *dev)
{
	unsigned int reg = 0;

	/*  Wait for serdes to be synced */
	while (!fmc_adc_get_serdes_sync_stat(dev));

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	reg &= ~FADC_FSM_CMD_MASK;
	reg |= ((FADC_FSM_CMD_START << FADC_CTL_FSM_CMD) & FADC_FSM_CMD_MASK);
	reg &= (FADC_CTL_MASK | FADC_FSM_CMD_MASK);
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
}

/*  Stop acquisition */
void fmc_adc_stop_acq(struct fadc_dev *dev)
{
	unsigned int reg = 0;

	reg = readl(FADC_CSR(dev, FADC_R_CTL));
	reg &= ~FADC_FSM_CMD_MASK;
	reg |= ((FADC_FSM_CMD_STOP << FADC_CTL_FSM_CMD) & FADC_FSM_CMD_MASK);
	writel(reg, FADC_CSR(dev, FADC_R_CTL));
}

/*  Get acquisition state machine status */
int fmc_adc_get_acq_fsm_state(struct fadc_dev *dev)
{
	unsigned int state = 0;

	state = readl(FADC_CSR(dev, FADC_R_STA)) & FADC_FSM_MASK;

	return state;
}

/*  Software trigger */
void fmc_adc_sw_trig(struct fadc_dev *dev)
{
	while (fmc_adc_get_acq_fsm_state(dev) != FADC_FSM_STATE_WAIT_TRIG);
	writel(0xFFFFFFFF, FADC_CSR(dev, FADC_R_SW_TRIG));
}

/*  Software trigger without wait on WAIT_TRIG state */
void fmc_adc_sw_trig_no_wait(struct fadc_dev *dev)
{
	writel(0xFFFFFFFF, FADC_CSR(dev, FADC_R_SW_TRIG));
}

/*  Set pre-trigger samples */
void fmc_adc_set_pretrig_samples(struct fadc_dev *dev, int samples)
{
	writel(samples, FADC_CSR(dev, FADC_R_PRE_SAMPLES));
}

/*  Set post-trigger samples */
void fmc_adc_set_posttrig_samples(struct fadc_dev *dev, int samples)
{
	writel(samples, FADC_CSR(dev, FADC_R_POST_SAMPLES));
}

/*  Set number of shots */
void fmc_adc_set_shots(struct fadc_dev *dev, int shots)
{
	writel(shots, FADC_CSR(dev, FADC_R_SHOTS));
}

/*  Get trigger position (DDR address) */
int fmc_adc_get_trig_pos(struct fadc_dev *dev)
{
	return readl(FADC_CSR(dev, FADC_R_TRIG_POS));
}

/*  Get number of acquired samples */
int fmc_adc_get_nb_samples(struct fadc_dev *dev)
{
	return readl(FADC_CSR(dev, FADC_R_SAMP_CNT));
}

/*  Get ADC core status */
int fmc_adc_get_status(struct fadc_dev *dev)
{
	return readl(FADC_CSR(dev, FADC_R_STA));
}

/*  Get Channel current ADC value */
int fmc_adc_get_curr_adc_val(struct fadc_dev *dev, int channel)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_VALUE);
	return readl(FADC_CSR(dev, addr));
}

/*  Set channel gain and offset correction */
void fmc_adc_set_gain_off_corr(struct fadc_dev *dev, int channel, int gain,
	int offset)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_GAIN);
	writel(gain, FADC_CSR(dev, addr));
	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_OFFSET);
	writel(offset, FADC_CSR(dev, addr));
}

/*  Get channel gain correction */
int fmc_adc_get_gain_corr(struct fadc_dev *dev, int channel)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_GAIN);

	return readl(FADC_CSR(dev, addr));
}

/*  Get channel offset correction */
int fmc_adc_get_off_corr(struct fadc_dev *dev, int channel)
{
	unsigned int addr;

	addr = fmc_adc_channel_addr(channel, FADC_R_CH1_OFFSET);

	return readl(FADC_CSR(dev, addr));
}

#if 0
def print_adc_core_config(self):
	print("\nADC core configuration/status")
	self.fmc_adc_csr.rd_reg(0x04) /*  Workaround for first read at 0x00 bug */
	for i in range(0,0x6C,4):
		print("%31s: 0x%.8X (%d)") % (self.FMC_CSR[i],self.fmc_adc_csr.rd_reg(i),self.fmc_adc_csr.rd_reg(i))

#endif

int fmc_adc_init(struct fadc_dev *dev)
{
	/* Set gain of all channels to 1 */
	fmc_adc_set_gain(dev, FMC_ADC_CHANNEL_1, 0x8000);
	fmc_adc_set_gain(dev, FMC_ADC_CHANNEL_2, 0x8000);
	fmc_adc_set_gain(dev, FMC_ADC_CHANNEL_3, 0x8000);
	fmc_adc_set_gain(dev, FMC_ADC_CHANNEL_4, 0x8000);

	/* Enable mezzanine clock and offset DACs */
	writel(((1 << FADC_CTL_CLK_EN) | (1 << FADC_CTL_OFFSET_DAC_CLR_N)),
			FADC_CSR(dev, FADC_R_CTL));

	return 0;
}

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

static long fadc_device_ioctl(struct file *f, unsigned int cmd,
	unsigned long arg)
{
	struct fadc_dev *fadcdev;

	fadcdev = f->private_data;

	return 0;
}

struct file_operations fadc_fops = {
	.owner = THIS_MODULE,
	.open = fadc_device_open,
	.release = fadc_device_release,
	.read = fadc_device_read,
	.write = fadc_device_write,
	.unlocked_ioctl = fadc_device_ioctl,
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
	ret = cdev_add(&fadcdev->cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "error %d adding cdev %d\n", ret, ndev);
		goto cdev_add_fail; 
	}

	fadcdev->dev = device_create(fadc_class, &pdev->dev, devno, fadcdev, "spec_adc%i", ndev);
	if (IS_ERR(fadcdev->dev)) {
		ret = PTR_ERR(fadcdev->dev);
		printk(KERN_ERR "error %d creating device %d\n", ret, ndev);
		goto device_create_fail;
	}

	fmc_adc_init(fadcdev);

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

	device_destroy(fadc_class, MKDEV(MAJOR(fadc_devno), fadcdev->ndev));
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
	ret = alloc_chrdev_region(&fadc_devno, 0, FMC_ADC_MAX_DEVICES, "spec_adc");
	if (ret) {
		printk(KERN_ERR "alloc_chrdev_region failed\n");
		goto alloc_chr_fail;
	}

	/* create a sysfs class */
	fadc_class = class_create(THIS_MODULE, "spec_adc");
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

	class_destroy(fadc_class);
	unregister_chrdev_region(fadc_devno, FMC_ADC_MAX_DEVICES);
	platform_driver_unregister(&fadc_driver);
}

module_init(fadc_init);
module_exit(fadc_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC+ADC Linux Driver");
MODULE_LICENSE("GPL");
