/*
 * A tool to program our soft-core (LM32) within the SPEC.
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
#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "spec-tools.h"

int main(int argc, char **argv)
{
	int i, fd;
	FILE *f;
	uint32_t base;
	struct stat stbuf;
	void *map_base;
	unsigned char *buf;
	unsigned int *ibuf;
	volatile uint32_t *p;

	if (argc != 2) {
		fprintf(stderr,
			"Use: \"%s <program>\"\n", argv[0]);
		exit(1);
	}

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		fprintf(stderr, "%s: open(/dev/mem): %s\n", argv[0],
			strerror(errno));
		exit(1);
	}

	base = spec_get_base();
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

	close(fd);

	f = fopen(argv[1], "r");
	if (!f) {
		fprintf(stderr, "%s: %s: %s\n", argv[0], argv[1],
			strerror(errno));
		exit(1);
	}
	if (fstat(fileno(f), &stbuf)) {
		fprintf(stderr, "%s: stat(%s): %s\n", argv[0], argv[1],
			strerror(errno));
		exit(1);
	}
	if (!S_ISREG(stbuf.st_mode)) {
		fprintf(stderr, "%s: %s: Not a regular file\n", argv[0],
			argv[1]);
		exit(1);
	}
	buf = malloc(stbuf.st_size);
	if (!buf) {
		fprintf(stderr, "%s: Can't allocate buffer (%li bytes): %s\n",
			argv[0], (long)stbuf.st_size, strerror(errno));
		exit(1);
	}
	i = fread(buf, 1, stbuf.st_size, f);
	if (i < 0) {
		fprintf(stderr, "%s: read(%s): %s\n", argv[0], argv[1],
			strerror(errno));
		exit(1);
	}
	if (i != stbuf.st_size) {
		fprintf(stderr, "%s: short read from %s\n", argv[0], argv[1]);
		exit(1);
	}
	ibuf = (void *)buf;

	/* Phew... we are there, finally */
	*(volatile uint32_t *)(map_base + 0xA0400) = 0x1deadbee;
	while ( !((*(volatile uint32_t *)(map_base + 0xA0400)) & (1<<28)) )
		  ;

	p = map_base + 0x80000;
 	for (i = 0; i < (stbuf.st_size + 3) / 4; i++) {
		p[i] = htonl(ibuf[i]); /* big endian */
		sync();
	}

	for (i = 0; i < (stbuf.st_size + 3) / 4; i++) {
		if (p[i] != htonl(ibuf[i]))
			fprintf(stderr, "programming error at %x "
				"(expected %08x, found %08x)\n", i*4,
				htonl(ibuf[i]), p[i]);
	}

	*(volatile uint32_t *)(map_base + 0xA0400) = 0x0deadbee;

	if (getenv("VERBOSE"))
		printf("%s: Wrote %li bytes at offset 0x8000\n", argv[0],
		       (long)stbuf.st_size);
	exit (0);
}
