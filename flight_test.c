
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <endian.h>

static const char *spiDevice = "/dev/spidev1.0";
static const char *gpioDevice = "/sys/class/gpio/gpio199/value";
static const char *logPath = "log.csv";
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 6000000;
static uint16_t delay = 0;
#define MESSAGE_LENGTH  (18*2) // flight controller sends 16bit bytes

static int gpioFd, spiFd, logFd;


float unpackFloat( uint8_t* from ) {
	uint32_t tmp = (from[ 3] << 0) | (from[ 2] << 8) | (from[1] << 16) | (from[0] << 24);
	return *(float*)&tmp;
}
void unpackFloats( uint8_t* from, float to[], uint32_t n ) {
	uint32_t i;
	for( i = 0; i < n; i++ ) {
		to[i] = unpackFloat( from );
		from += 4;
	}
}

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


	float acc[3];
	unpackFloats( &rx[0], acc, 3 );
	float gyro[3];
	unpackFloats( &rx[sizeof(acc)], gyro, 3 );
	float alpha[3];
	unpackFloats( &rx[sizeof(acc)+sizeof(gyro)], alpha, 3 );

	/*
	printf( "Acc: %3.3f\n     %3.3f\n     %3.3f\n",
		acc[0], acc[1], acc[2] );
	printf( "Gyro: %3.3f\n      %3.3f\n      %3.3f\n",
		gyro[0], gyro[1], gyro[2] );
	printf( "Alpha: %3.3f\n       %3.3f\n       %3.3f\n",
		alpha[0], alpha[1], alpha[2] );
	*/
	struct timeval tv;
		struct timezone tz = {
		.tz_minuteswest = 0,
		.tz_dsttime = 0
	};
	gettimeofday( &tv, &tz );

	dprintf( logFd, "%ld.%ld,  %f, %f, %f,  %f, %f, %f,  %f, %f, %f\n",
		tv.tv_sec, tv.tv_usec,
		acc[0], acc[1], acc[2],
		gyro[0], gyro[1], gyro[2],
		alpha[0], alpha[1], alpha[2] );
	/*
	for ( ret = 0; ret < MESSAGE_LENGTH; ret++ ) {
		if ( !( ret % 6 ) )
			puts( "" );
		printf( "%.2X ", rx[ret] );
	}
	puts( "" );
	*/
}

int main( int argc, char *argv[] ) {
	float tmp = 0.12584944;
	printf( "0x%x\n",*(unsigned int*)(&tmp) );
	int ret; // return conde for spi calls

	// Open log file and check for error
	logFd = open( logPath, O_WRONLY | O_CREAT );
	if ( logFd < 0 ) {
		perror( "log file open" );
		return EXIT_FAILURE;
	}
	fchmod( logFd, 0666 );

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
	close (logFd);

}
