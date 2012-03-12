
/* Just a test to check it runs. Keep changing a meory location */

volatile int *p = (volatile int *)0x8000; /* static addr: easily found */

int main(void)
{
	*p = 0;
	while (1)
		(*p)++;
}


/* needed ... */
void _irq_entry(void)
{}
