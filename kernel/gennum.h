#ifndef SPEC_ADC_GENNUM_H
#define SPEC_ADC_GENNUM_H

#include "spec.h"
#include "fmcadc.h"

/* GN4124 interrupt configuration */
void gn_set_interrupt_config(struct spec_dev *spec);

/* Get DMA controller status */
int gn_get_dma_status(struct spec_dev *spec);

/*
 * Configure DMA byte swapping
 *   0 = A1 B2 C3 D4 (straight)
 *   1 = B2 A1 D4 C3 (swap bytes in words)
 *   2 = C3 D4 A1 B2 (swap words)
 *   3 = D4 C3 B2 A1 (invert bytes)
 */
int gn_set_dma_swap(struct spec_dev *spec, int swap);


/*
 * Add DMA item (first item is on the board, the following in the host memory)
 *   carrier_addr, host_addr, length and next_item_addr are in bytes
 *   dma_dir  = 1 -> PCIe to carrier
 *   dma_dir  = 0 -> carrier to PCIe
 *   dma_last = 0 -> last item in the transfer
 *   dma_last = 1 -> more item in the transfer
 *   Only supports 32-bit host address
 */
void gn_add_dma_item(struct fadc_dev *fadcdev, unsigned int carrier_addr,
	unsigned int host_addr, int length, int dma_dir, int last_item);

/* Start DMA transfer */
void gn_start_dma(struct spec_dev *spec);

/* Abort DMA transfer */
void gn_abort_dma(struct spec_dev *spec);

#endif
