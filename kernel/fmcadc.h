#ifndef LINUX_SPEC_FMC_ADC_H
#define LINUX_SPEC_FMC_ADC_H

#define FADC_FMC_SYS_I2C_ADDR	0x60000
#define FADC_EEPROM_ADDR	0x50

#define FADC_FMC_SPI_ADDR	0x70000
#define FADC_FMC_SPI_DIV	100
#define FADC_FMC_SPI_SS_ADC	0
#define FADC_FMC_SPI_SS_DAC1	1
#define FADC_FMC_SPI_SS_DAC2	2
#define FADC_FMC_SPI_SS_DAC3	3
#define FADC_FMC_SPI_SS_DAC4	4
#define FADC_FMC_I2C_ADDR	0x80000
#define FADC_SI570_ADDR		0x55

#define FADC_FMC_ONEWIRE_ADDR	0xA0000


#define FADC_FMC_CSR_ADDR	0x90000
#define FADC_R_CTL		0x00
#define FADC_R_STA		0x04
#define FADC_R_TRIG_CFG		0x08
#define FADC_R_TRIG_DLY		0x0C
#define FADC_R_SW_TRIG		0x10
#define FADC_R_SHOTS		0x14
#define FADC_R_TRIG_POS		0x18
#define FADC_R_SRATE		0x1C
#define FADC_R_PRE_SAMPLES	0x20
#define FADC_R_POST_SAMPLES	0x24
#define FADC_R_SAMP_CNT		0x28
#define FADC_R_CH1_SSR		0x2C
#define FADC_R_CH1_VALUE	0x30
#define FADC_R_CH1_GAIN		0x34
#define FADC_R_CH1_OFFSET	0x38
#define FADC_R_CH2_SSR		0x3C
#define FADC_R_CH2_VALUE	0x40
#define FADC_R_CH2_GAIN		0x44
#define FADC_R_CH2_OFFSET	0x48
#define FADC_R_CH3_SSR		0x4C
#define FADC_R_CH3_VALUE	0x50
#define FADC_R_CH3_GAIN		0x54
#define FADC_R_CH3_OFFSET	0x58
#define FADC_R_CH4_SSR		0x5C
#define FADC_R_CH4_VALUE	0x60
#define FADC_R_CH4_GAIN		0x64
#define FADC_R_CH4_OFFSET	0x68

#define FADC_CTL_FSM_CMD		0
#define FADC_CTL_CLK_EN			2
#define FADC_CTL_OFFSET_DAC_CLR_N	3
#define FADC_CTL_BSLIP			4
#define FADC_CTL_TEST_DATA_EN		5
#define FADC_CTL_TRIG_LED		6
#define FADC_CTL_ACQ_LED		7
#define FADC_CTL_MASK			0xEC

#define FADC_FSM_CMD_MASK		0x00000003
#define FADC_FSM_CMD_START		0x1
#define FADC_FSM_CMD_STOP		0x2

#define FADC_STA_FSM			0
#define FADC_STA_SERDES_SYNCED		4

#define FADC_FSM_MASK			0x00000007

#define FADC_FSM_STATE_NA		0
#define FADC_FSM_STATE_IDLE		1
#define FADC_FSM_STATE_PRE_TRIG		2
#define FADC_FSM_STATE_POST_TRIG	3
#define FADC_FSM_STATE_WAIT_TRIG	4
#define FADC_FSM_STATE_DECR_SHOT	5
#define FADC_FSM_STATE_NA2		6
#define FADC_FSM_STATE_OTHER		7

#define FADC_TRIG_CFG_HW_SEL		0
#define FADC_TRIG_CFG_EXT_POL		1
#define FADC_TRIG_CFG_HW_EN		2
#define FADC_TRIG_CFG_SW_EN		3
#define FADC_TRIG_CFG_INT_SEL		4
#define FADC_TRIG_CFG_INT_THRES		16

#define FADC_INT_SEL_MASK		0xFFFFFFCF
#define FADC_INT_THRES_MASK		0x0000FFFF

#define FADC_IN_TERM			(1<<3)
#define FADC_IN_TERM_MASK		0x08

#define FADC_RANGE_100mV		0x23
#define FADC_RANGE_1V			0x11
#define FADC_RANGE_10V			0x45
#define FADC_RANGE_CAL			0x40
#define FADC_RANGE_OPEN			0x0
#define FADC_RANGE_CAL_100mV		0x42
#define FADC_RANGE_CAL_1V		0x40
#define FADC_RANGE_CAL_10V		0x44

struct fadc_dev;

void fmc_adc_set_gain(struct fadc_dev *dev, int channel, int value);
void fadc_trig_led(struct fadc_dev *dev, int state);
void fadc_acq_led(struct fadc_dev *dev, int state);
void fmc_adc_set_input_range(struct fadc_dev *dev, int channel, int in_range);
/* Set SSR register */
void fmc_adc_set_ssr(struct fadc_dev *dev, int ch, int val);
/* Get SSR register */
unsigned int fmc_adc_get_ssr(struct fadc_dev *dev, int ch);
/* Set trigger configuration */
void fmc_adc_set_trig_config(struct fadc_dev *dev, unsigned int hw_sel,
	unsigned int ext_pol, unsigned int hw_en, unsigned int sw_en,
	unsigned int int_sel, unsigned int int_thres, unsigned int delay);
/*  Internal trigger */
void fmc_adc_set_int_trig(struct fadc_dev *dev, int channel, int polarity,
	int threshold, int int_sel);
/*  External trigger */
void fmc_adc_set_ext_trig(struct fadc_dev *dev, int polarity);
/*  Software trigger */
void fmc_adc_set_soft_trig(struct fadc_dev *dev);
/*  Trigger delay */
void fmc_adc_set_trig_delay(struct fadc_dev *dev, int delay);
/*  Enable test data */
void fmc_adc_test_data_en(struct fadc_dev *dev);
/*  Disable test data */
void fmc_adc_test_data_dis(struct fadc_dev *dev);
/*  Get serdes sync status */
int fmc_adc_get_serdes_sync_stat(struct fadc_dev *dev);
/*  Start acquisition */
void fmc_adc_start_acq(struct fadc_dev *dev);
/*  Stop acquisition */
void fmc_adc_stop_acq(struct fadc_dev *dev);
/*  Get acquisition state machine status */
int fmc_adc_get_acq_fsm_state(struct fadc_dev *dev);
/*  Software trigger */
void fmc_adc_sw_trig(struct fadc_dev *dev);
/*  Software trigger without wait on WAIT_TRIG state */
void fmc_adc_sw_trig_no_wait(struct fadc_dev *dev);
/*  Set pre-trigger samples */
void fmc_adc_set_pretrig_samples(struct fadc_dev *dev, int samples);
/*  Set post-trigger samples */
void fmc_adc_set_posttrig_samples(struct fadc_dev *dev, int samples);
/*  Set number of shots */
void fmc_adc_set_shots(struct fadc_dev *dev, int shots);
/*  Get trigger position (DDR address) */
int fmc_adc_get_trig_pos(struct fadc_dev *dev);
/*  Get number of acquired samples */
int fmc_adc_get_nb_samples(struct fadc_dev *dev);
/*  Get ADC core status */
int fmc_adc_get_status(struct fadc_dev *dev);
/*  Get Channel current ADC value */
int fmc_adc_get_curr_adc_val(struct fadc_dev *dev, int channel);
/*  Set channel gain and offset correction */
void fmc_adc_set_gain_off_corr(struct fadc_dev *dev, int channel, int gain,
	int offset);
/*  Get channel gain correction */
int fmc_adc_get_gain_corr(struct fadc_dev *dev, int channel);
/*  Get channel offset correction */
int fmc_adc_get_off_corr(struct fadc_dev *dev, int channel);

/* ioctl commands */
#define __FADC_IOC_MAGIC '4' /* random or so */

#define FADC_ACQUIRE		_IOW(__FADC_IOC_MAGIC, 0, unsigned char *)

#endif
