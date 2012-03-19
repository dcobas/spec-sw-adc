
uint32_t spec_get_base(int basenr);

enum {
	BASE_BAR0 = 0,	/* for wrpc etc (but lm32 is at 0x80000 offset) */
	BASE_BAR2,
	BASE_BAR4	/* for gennum-internal registers */
};

