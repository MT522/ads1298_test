/*
 * Minimal test: read a few 27-byte packets from /dev/ads1298 and print hex + status.
 * Build: gcc -o test_read test_read.c
 * Run: sudo ./test_read [num_packets]
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define PACKET_SIZE 27
#define DEFAULT_COUNT 10

int main(int argc, char **argv)
{
	int fd, n, count = DEFAULT_COUNT;
	unsigned char buf[PACKET_SIZE];
	ssize_t got;

	if (argc >= 2)
		count = atoi(argv[1]);
	if (count <= 0)
		count = DEFAULT_COUNT;

	fd = open("/dev/ads1298", O_RDONLY);
	if (fd < 0) {
		perror("open /dev/ads1298");
		return 1;
	}

	printf("Reading %d packets (%d bytes) from /dev/ads1298\n", count, count * PACKET_SIZE);
	for (n = 0; n < count; n++) {
		got = read(fd, buf, PACKET_SIZE);
		if (got != PACKET_SIZE) {
			if (got < 0)
				perror("read");
			break;
		}
		printf("Packet %d: status %02X %02X %02X | ch1..8 (24b MSB first) ...\n",
		       n + 1, buf[0], buf[1], buf[2]);
	}

	close(fd);
	return 0;
}
