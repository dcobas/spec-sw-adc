/*
 * A tool to read SPEC-internal memory (only BAR0)
 *
 * Alessandro Rubini 2012 for CERN, GPLv2 or later.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "spec-tools.h"

int main(int argc, char **argv)
{
	int i, fd, bar = BASE_BAR0;
	uint32_t base, *ptr;
	uint32_t uarg[3];
	void *map_base;
	char *end;

	if (argc > 1 && !strcmp(argv[1], "-g")) {
		bar = BASE_BAR4;
		argv[1] = argv[0];
		argc--; argv++;
	}

	if (argc < 2 || argc > 3) {
		fprintf(stderr,
			"Use: \"%s [-g] <offset> [<value>]\" "
			"(-g selects gennum memory, I/O is 32 bits)\n",
			argv[0]);
		exit(1);
	}

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		fprintf(stderr, "%s: open(/dev/mem): %s\n", argv[0],
			strerror(errno));
		exit(1);
	}

	base = spec_get_base(bar);
	if (base == (typeof(base))-1) {
		fprintf(stderr, "%s: spec_get_base(): %s\n", argv[0],
			strerror(errno));
		exit(1);
	}

	map_base = mmap(0, 1024 * 1024, /* gennum's bar 0 is 1M */
			PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);

	if(map_base == (void *) -1) {
		fprintf(stderr, "%s: mmap(/dev/mem): %s\n", argv[0],
			strerror(errno));
		exit(1);
	}

	for (i = 1; i < argc; i++) {
		uarg[i] = strtol(argv[i], &end, 16);
		if (end && *end) {
			fprintf(stderr, "%s: \"%s\" is not an hex number\n",
				argv[0], argv[i]);
			exit(1);
		}
	}
	if (uarg[1] & 3) {
		fprintf(stderr, "%s: address \"%s\" not multiple of 4\n",
			argv[0], argv[1]);
		exit(1);
	}

	ptr = map_base + uarg[1];

	/* by default, operate quietly (only report read value) */
	if (argc == 2) {
		uarg[2] = *ptr;
		if (!getenv("VERBOSE"))
			printf("%08x\n", uarg[2]);
	} else {
		*ptr = uarg[2];
	}

	/* be verbose, if so requested */
	if (getenv("VERBOSE")) {
		if (argc == 2)
			printf("%08x == %08x\n", uarg[1], uarg[2]);
		else
			printf("%08x := %08x\n", uarg[1], uarg[2]);
	}

	exit (0);
}
