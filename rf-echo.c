#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "linux/nrf24.h"

#define die(fmt, args...) \
	({fprintf(stderr, "%d/%s: " fmt "\n", errno, strerror(errno), ##args); exit(EXIT_FAILURE);})

static int convert_hexstr_to_binary(char *str, char *buf, int blen)
{
	int i;
	assert(!(strlen(str) % 2)); /* str should have even length */
	for (i = 0; i < blen; i++)
		sscanf(&str[i*2], "%2hhx", &buf[i]);
	return i;
}

int main(int argc, char **argv)
{
	size_t aw;
	unsigned char rxaddr0[5], rxaddr1[5], rxaddr2, rxaddr3, rxaddr4, rxaddr5;
	int fd = open("/dev/nrf24-0.0", O_RDWR);

	if (argc != 7) {
		fprintf(stderr, "Usage: %s RXADDR0 RXADDR1 RXADDR2 RXADDR3 RXADDR4 RXADDR5 RXADDR6\n"
				"Where: 3 bytes >= RXADDR0 & RXADDR1 <= 5 bytes\n"
				"       RXADDR2 ... RXADDR4 == 1 byte\n",
				argv[0]);
		exit(EXIT_FAILURE);
	}

	aw = strlen(argv[1]) / 2;

	if (fd == -1)
		die("open: /dev/nrf24-0.0");

	convert_hexstr_to_binary(argv[1], rxaddr0, aw);
	convert_hexstr_to_binary(argv[2], rxaddr1, aw);
	rxaddr2 = strtoul(argv[3], NULL, 16);
	rxaddr3 = strtoul(argv[4], NULL, 16);
	rxaddr4 = strtoul(argv[5], NULL, 16);
	rxaddr5 = strtoul(argv[6], NULL, 16);

        ioctl(fd, NRF24_IOCSCRC,          2); /* setup CRC to 2 bytes */
        ioctl(fd, NRF24_IOCSAA,        0x00); /* disable auto-acknowledge */
        ioctl(fd, NRF24_IOCSAW,          aw); /* address width X bytes */
        ioctl(fd, NRF24_IOCSRETR,         0); /* disable auto-retrasmit */
        ioctl(fd, NRF24_IOCSCH,           2); /* setup default channel to 2 */
        ioctl(fd, NRF24_IOCSRF,        0x26); /* setup data-rate &  power */
        ioctl(fd, NRF24_IOCSPW,          32); /* setup payload size (32 bytes) */
	ioctl(fd, NRF24_IOCSENRXADDR,   0x3f); /* enable all data pipes */
	ioctl(fd, NRF24_IOCSFEATURE,   0x04); /* enable dynamic payload feature */
        ioctl(fd, NRF24_IOCSDYNPLD,    0x3f); /* enable dynamic payload on all pipes */
	ioctl(fd, NRF24_IOCSRXADDR,    rxaddr0);
	ioctl(fd, NRF24_IOCSRXADDR_P1, rxaddr1);
	ioctl(fd, NRF24_IOCSRXADDR_P2, rxaddr2);
	ioctl(fd, NRF24_IOCSRXADDR_P3, rxaddr3);
	ioctl(fd, NRF24_IOCSRXADDR_P4, rxaddr4);
	ioctl(fd, NRF24_IOCSRXADDR_P5, rxaddr5);

	for (;;) {
		unsigned char rxpld[32];
		unsigned char rxaddr[5];
		struct nrf_read_multiceiver m;
		int status;

		m.timeout = 0;
		m.rx      = rxpld;
		m.rx_addr = rxaddr;

		status = ioctl(fd, NRF24_IOREADMULTICEIVER, &m);
		if (!status) {
			int i;
			printf("(");
			for (i = 0; i < aw; i++)
				printf("%02x", m.rx_addr[i]);
			printf(") ");

			for (i = 0; i < m.rx_siz; i++)
				printf("%02x:", rxpld[i]);
			putchar('\n');

			ioctl(fd, NRF24_IOCSTXADDR, m.rx_addr);
			write(fd, rxpld, m.rx_siz);
		} else
			printf("ioctl error: %s", strerror(errno));
	}

}
