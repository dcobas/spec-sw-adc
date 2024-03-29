/*
 * Trivial library function to return one of the spec memory addresses
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "spec-tools.h"

uint32_t spec_get_base(int base)
{
	FILE *f;
	int found = 0;
	unsigned int res = ~0;
	char s[160];

	f = popen("lspci -v", "r");
	while(fgets(s, sizeof(s), f)) {
		if (!found) {
			if (strstr(s, "Device 1a39:0004"))
				found = 1;
			if (strstr(s, "Device 10dc:018d"))
				found = 1;
		}
		if (found && sscanf(s, " Memory at %x", &res) == 1) {
			if (!base)
				break;
			base--;
		}
	}
	pclose(f);
	if (res == ~0)
		errno = ENODEV;
	return res;
}
