#include "spec.h"
#include "fmcadc.h"

#define HOST_BAR				0xC 
#define HOST_DMA_CARRIER_START_ADDR		0x00
#define HOST_DMA_HOST_START_ADDR_L		0x04
#define HOST_DMA_HOST_START_ADDR_H		0x08
#define HOST_DMA_LENGTH				0x0C
#define HOST_DMA_NEXT_ITEM_ADDR_L		0x10
#define HOST_DMA_NEXT_ITEM_ADDR_H		0x14
#define HOST_DMA_ATTRIB				0x18

#define GN4124_BAR				0x4 
#define R_PCI_SYS_CFG				0x800
#define R_CLK_CSR				0x808
#define R_INT_CFG0				0x820
#define R_GPIO_DIR_MODE				0xA04
#define R_GPIO_INT_MASK_CLR			0xA18
#define R_GPIO_INT_MASK_SET			0xA1C
#define R_GPIO_INT_STATUS			0xA20
#define R_GPIO_INT_VALUE			0xA28

#define CLK_CSR_DIVOT_MASK			0x3F0
#define INT_CFG0_GPIO				15
#define GPIO_INT_SRC				8 

#define R_DMA_CTL				0x00
#define R_DMA_STA				0x04
#define R_DMA_CARRIER_START_ADDR		0x08
#define R_DMA_HOST_START_ADDR_L			0x0C
#define R_DMA_HOST_START_ADDR_H			0x10
#define R_DMA_LENGTH				0x14
#define R_DMA_NEXT_ITEM_ADDR_L			0x18
#define R_DMA_NEXT_ITEM_ADDR_H			0x1C
#define R_DMA_ATTRIB				0x20

#define DMA_CTL_START				0 
#define DMA_CTL_ABORT				1 
#define DMA_CTL_SWAP				2 
#define DMA_STATUS_IDLE				0
#define DMA_STATUS_DONE				1
#define DMA_STATUS_BUSY				2
#define DMA_STATUS_ERROR			3
#define DMA_STATUS_ABORTED			4
#define DMA_ATTRIB_DIR				1 


/* GN4124 interrupt configuration */
void gn_set_interrupt_config(struct spec_dev *spec)
{
	void *bar4 = spec->remap[2];

	/* Set interrupt line from FPGA (GPIO8) as input */
	writel(1<<GPIO_INT_SRC, bar4 + R_GPIO_DIR_MODE);
	/* Set interrupt mask for all GPIO except for GPIO8 */
	writel(~(1<<GPIO_INT_SRC), bar4 + R_GPIO_INT_MASK_SET);
	/* Make sure the interrupt mask is cleared for GPIO8 */
	writel(1<<GPIO_INT_SRC, bar4 + R_GPIO_INT_MASK_CLR);
	/* Interrupt on rising edge of GPIO8 */
	writel(1<<GPIO_INT_SRC, bar4 + R_GPIO_INT_VALUE);
	/* GPIO as interrupt 0 source */
	writel(1<<INT_CFG0_GPIO, bar4 + R_INT_CFG0);
}

/* Get DMA controller status */
int gn_get_dma_status(struct spec_dev *spec)
{
	void *bar0 = spec->remap[0];
	unsigned int reg;

	reg = readl(bar0 + R_DMA_STA);
	if (reg > DMA_STATUS_ABORTED)
		return -1;
	return reg;
}

/*
 * Configure DMA byte swapping
 *   0 = A1 B2 C3 D4 (straight)
 *   1 = B2 A1 D4 C3 (swap bytes in words)
 *   2 = C3 D4 A1 B2 (swap words)
 *   3 = D4 C3 B2 A1 (invert bytes)
 */
int gn_set_dma_swap(struct spec_dev *spec, int swap)
{
	void *bar0 = spec->remap[0];

	if (swap > 3)
		return -EINVAL;
	writel((swap << DMA_CTL_SWAP), bar0 + FADC_R_CTL);
	return 0;
}


/*
 * Add DMA item (first item is on the board, the following in the host memory)
 *   carrier_addr, host_addr, length and next_item_addr are in bytes
 *   dma_dir  = 1 -> PCIe to carrier
 *   dma_dir  = 0 -> carrier to PCIe
 *   dma_last = 0 -> last item in the transfer
 *   dma_last = 1 -> more item in the transfer
 *   Only supports 32-bit host address
 */
int dma_item_count = 0;
#if 0
void gn_add_dma_item(struct spec_dev *spec, unsigned int carrier_addr,
	unsigned int host_addr, int length, int dma_dir, int last_item)
{
	void *bar0 = spec->remap[0];
	unsigned int attrib;

	if (dma_item_count == 0) {
		/* write the first DMA item in the carrier */
		writel(carrier_addr, bar0 + R_DMA_CARRIER_START_ADDR);
		writel((host_addr & 0xFFFFFFFF), bar0 + R_DMA_HOST_START_ADDR_L);
		writel((host_addr >> 32), bar0 + R_DMA_HOST_START_ADDR_H);
		writel(length, bar0 + R_DMA_LENGTH);
		writel((self.pages[0] & 0xFFFFFFFF), bar0 + R_DMA_NEXT_ITEM_ADDR_L);
		writel(0x0, bar0 + R_DMA_NEXT_ITEM_ADDR_H);
		attrib = (dma_dir << self.DMA_ATTRIB_DIR) + (last_item << self.DMA_ATTRIB_LAST)
		writel(attrib, bar0 + R_DMA_ATTRIB);
	} else {
		/*
		 * write nexy DMA item(s) in host memory
		 * uses page 0 to store DMA items
		 * current and next item addresses are automatically set
		 */
		current_item_addr = (dma_item_count-1)*0x20
		next_item_addr = (dma_item_count)*0x20
		writel(self.HOST_BAR, self.HOST_DMA_CARRIER_START_ADDR + current_item_addr, carrier_addr)
		writel(self.HOST_BAR, self.HOST_DMA_HOST_START_ADDR_L + current_item_addr, host_addr)
		writel(self.HOST_BAR, self.HOST_DMA_HOST_START_ADDR_H + current_item_addr, 0x0)
		writel(self.HOST_BAR, self.HOST_DMA_LENGTH + current_item_addr, length)
		writel(self.HOST_BAR, self.HOST_DMA_NEXT_ITEM_ADDR_L + current_item_addr, self.pages[0] + next_item_addr)
		self.wr_reg(self.HOST_BAR, self.HOST_DMA_NEXT_ITEM_ADDR_H + current_item_addr, 0x0)
		attrib = (dma_dir << self.DMA_ATTRIB_DIR) + (last_item << self.DMA_ATTRIB_LAST)
		self.wr_reg(self.HOST_BAR, self.HOST_DMA_ATTRIB + current_item_addr, attrib)
		self.dma_item_count += 1
	}
}
#endif


/* Start DMA transfer */
void gn_start_dma(struct spec_dev *spec)
{
	unsigned int reg;

	dma_item_count = 0;

	reg = readl(spec->remap[0] + R_DMA_CTL);
	reg |= (1 << DMA_CTL_START);
	writel(reg, spec->remap[0] + R_DMA_CTL);

	/*
	 * The following two lines should be removed
	 * when the GN4124 vhdl core will implement auto clear of start bit
	 * while(('Idle' == self.get_dma_status()) or
	 *      ('Busy' == self.get_dma_status())):
	 *    pass
	 */
	while (gn_get_dma_status(spec) == DMA_STATUS_IDLE);

	reg = readl(spec->remap[0] + R_DMA_CTL);
	reg &= ~(1 << DMA_CTL_START);
	writel(reg, spec->remap[0] + R_DMA_CTL);
}

/* Abort DMA transfer */
void gn_abort_dma(struct spec_dev *spec)
{
	unsigned int reg;

	dma_item_count = 0;

	reg = readl(spec->remap[0] + R_DMA_CTL);
	reg |= (1 << DMA_CTL_ABORT);
	writel(reg, spec->remap[0] + R_DMA_CTL);

	/*
	 * The following two lines should be removed
	 * when the GN4124 vhdl core will implement auto clear of start bit
	 */
	while (gn_get_dma_status(spec) != DMA_STATUS_ABORTED);

	reg = readl(spec->remap[0] + R_DMA_CTL);
	reg &= ~(1 << DMA_CTL_ABORT);
	writel(reg, spec->remap[0] + R_DMA_CTL);
}

#if 0
/* Get memory page */
void gn_get_memory_page(self, page_nb):
	data = []
	for i in range(2**10):
		data.append(self.rd_reg(self.HOST_BAR, (page_nb<<12)+(i<<2)))
		return data

/* Set memory page */
def set_memory_page(self, page_nb, pattern):
	for i in range(2**10):
		self.wr_reg(self.HOST_BAR, (page_nb<<12)+(i<<2), pattern)
#endif
