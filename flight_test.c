
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
#include <endian.h>

static const char *spiDevice = "/dev/spidev1.0";
static const char *gpioDevice = "/sys/class/gpio/gpio199/value";
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 6000000;
static uint16_t delay = 0;
#define MESSAGE_LENGTH  (18*2) // flight controller sends 16bit bytes

static int gpioFd, spiFd;

void gpioIntHandler( void ) {
	int ret;
	uint8_t tx[MESSAGE_LENGTH] = { 0 };
	uint8_t rx[MESSAGE_LENGTH] = { 0 };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = MESSAGE_LENGTH,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};


	ret = ioctl( spiFd, SPI_IOC_MESSAGE(1), &tr );
	if ( ret < 1 )
		perror( "can't send spi message" );

	printf("Acc: %3.3f\n     %3.3f\n     %3.3f\n",
		*(float*)&le32toh( ((uint32_t*)rx)[0] ),
		*(float*)&le32toh( ((uint32_t*)rx)[1] ),
		*(float*)&le32toh( ((uint32_t*)rx)[2] ) );
	printf("Gyro: %3.3f\n      %3.3f\n      %3.3f\n",
		((float*)rx)[3],
		((float*)rx)[4],
		((float*)rx)[5] );
	printf("Alpha: %3.3f\n       %3.3f\n       %3.3f\n",
		((float*)rx)[6],
		((float*)rx)[7],
		((float*)rx)[8] );
	for ( ret = 0; ret < MESSAGE_LENGTH; ret++ ) {
		if ( !( ret % 6 ) )
			puts( "" );
		printf( "%.2X ", rx[ret] );
	}
	puts( "" );
}

int main( int argc, char *argv[] ) {
	int ret; // return conde for spi calls

	// Open gpio file and check for error
	gpioFd = open( gpioDevice, O_RDONLY | O_NONBLOCK );
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

	// clear interrupt
	char c;
	lseek ( gpioFd, 0, SEEK_SET);
	read( gpioFd, &c, 1);

	while( 1 ) {
		int rc; // return code
		struct pollfd fdset;
		fdset.fd = gpioFd;
		fdset.events = POLLPRI;
		fdset.revents = 0;


		// start waiting for next interrupt
		rc = poll( &fdset, 1, 500 );
		// clear interrupt
		lseek ( gpioFd, 0, SEEK_SET);
		read( gpioFd, &c, 1);

		if (rc < 0) {
			perror("poll() failed!");
			return EXIT_FAILURE;
		}

		if (rc == 0) {
			printf(".");
		}

		if (fdset.revents & POLLPRI) {
			printf("poll() GPIO: interrupt occurred\n");
			gpioIntHandler( );
		}

		fflush(stdout);
	}



	close(gpioFd);
	close(spiFd);

}
