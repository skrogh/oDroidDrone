#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

static const char *spiDevice = "/dev/spidev1.0";
static const char *gpioDevice = "/sys/class/gpio/gpio199";
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 6000000;
static uint16_t delay = 0;

int main( int argc, char *argv[] ) {
	int gpioFd, spiFd;
	int ret; // return conde for spi calls

	// Open gpio file and check for error
	File * f = fopen( gpioDevice, "r" );
	gpioFd = fileno(f);
	if ( gpioFd < 0 ) {
		perror( "gpio file open" );
		return EXIT_FAILURE;
	}

	// Open spi file and check for error
	spiFd = open( spiDevice, O_RDWR );
	if ( spiFd < 0 ) {
		perror( "spi file open" );
		return EXIT_FAILURE;
	}

	/*
	 * spi mode
	 */
	ret = ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		perror("can't set spi mode");
		return EXIT_FAILURE;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
		perror("can't get spi mode");
		return EXIT_FAILURE;
	}

	/*
	 * bits per word
	 */
	ret = ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		perror("can't set bits per word");
		return EXIT_FAILURE;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
		perror("can't get bits per word");
		return EXIT_FAILURE;
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		perror("can't set max speed hz");
		return EXIT_FAILURE;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		perror("can't get max speed hz");
		return EXIT_FAILURE;
	}

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);


	while( 1 ) {
		int rc; // return code
		struct pollfd fdset;
		fdset.fd = gpioFd;
		fdset.events = POLLPRI;
		fdset.revents = 0;

		char c;
		lseek ( gpioFd, 0, SEEK_SET);
		read( gpioFd, &c, 1);
		// start polling for next interrupt
		rc = poll( &fdset, 1, 500 );

		if (rc < 0) {
			perror("poll() failed!");
			return EXIT_FAILURE;
		}

		if (rc == 0) {
			printf(".");
		}

		if (fdset.revents & POLLPRI) {
			printf("poll() GPIO: interrupt occurred\n");
		}

		fflush(stdout);
	}



	close(gpioFd);
	close(spiFd);

}
