#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include "fmcadc.h"

int main(int argc, char *argv[])
{
	int fd; 
	int ret;
	FILE *file;
	int len;
	unsigned char buffer[4096];
	int num;
	int i;

	fd = open("/dev/spec_adc0", O_RDWR);
	if (fd < 0) {
		printf("failed to open file\n");
		return fd; 
	}

	if ((ret = ioctl(fd, FADC_ACQUIRE, buffer)) < 0) {
		printf("Error: %d\n", ret);
		return ret;
	}

	printf("data:\n");
	for (i = 0; i < 4096; i++) {
		if (i % 32 == 0 && i != 0)
			printf("\n");
		printf("%02x ", buffer[i]);
	}

	close(fd);
	return 0;
}

