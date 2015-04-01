#include "imu.hpp"

#define MESSAGE_LENGTH  (20*2) // flight controller sends 16bit bytes

uint32_t unpackUint32( uint8_t* from ) {
	return (from[ 3] << 0) | (from[ 2] << 8) | (from[1] << 16) | (from[0] << 24);
}

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

Imu::Imu( const char *spiDevice, const char *gpioDevice ) {
	//
	// Init variables
	//
	endThread = false;
	mode = 0;
	bits = 8;
	speed = 6000000;
	delay = 0;
	timeout = 500;

	//
	// open I/O files
	//

	// Open gpio file and check for error
	gpioFd = open( gpioDevice, O_RDONLY | O_NONBLOCK );
	if ( gpioFd < 0 ) {
		perror( "gpio file open" );
	}

	// Open spi file and check for error
	spiFd = open( spiDevice, O_RDWR );
	if ( spiFd < 0 ) {
		perror( "spi file open" );
	}

	//
	// Setup spi
	//
	int ret;
	// spi mode
	ret = ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		perror("can't set spi mode");
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
		perror("can't get spi mode");
	}

	//bits per word
	ret = ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		perror("can't set bits per word");
	}

	ret = ioctl(spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
		perror("can't get bits per word");
	}

	 // max speed hz
	ret = ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		perror("can't set max speed hz");
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		perror("can't get max speed hz");
	}

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//
	// start thread
	//
	pthread_create( &thread, NULL, &Imu::imuThread, this );
}

Imu::~Imu( ) {
	endThread = true;
	pthread_join( thread, NULL );

	close(gpioFd);
	close(spiFd);
}

void inline Imu::clearSpiInt( void ) {
	// clear interrupt
	char c;
	lseek ( gpioFd, 0, SEEK_SET);
	read( gpioFd, &c, 1);
}

void Imu::imuThread( void *pointerToThis ) {
	// Lazy conversion
	Imu* This = (Imu* )pointerToThis;

	int rc; // return code
	struct pollfd fdset;
	fdset.fd = This->gpioFd;
	fdset.events = POLLPRI;

	This->clearSpiInt();

	while( !endThread ) {
		fdset.revents = 0;

		// start waiting for next interrupt
		rc = poll( &fdset, 1, This->timeout );
		// clear interrupt
		This->clearSpiInt();

		// Check if interrupt request failed
		if (rc < 0) {
			perror("poll() failed!");
		}

		// Check if timed out
		if (rc == 0) {
		}

		// Check if correct interrupt
		if (fdset.revents & POLLPRI) {
			this->gpioIntHandler( );
		}
	}
	pthread_exit(NULL);
}

void Imu::gpioIntHandler( void ) {
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
	uint32_t ping = unpackUint32( &rx[sizeof(acc)+sizeof(gyro)+sizeof(alpha)] );
	
	printf( "Acc: %3.3f\n     %3.3f\n     %3.3f\n",
		acc[0], acc[1], acc[2] );
	printf( "Gyro: %3.3f\n      %3.3f\n      %3.3f\n",
		gyro[0], gyro[1], gyro[2] );
	printf( "Alpha: %3.3f\n       %3.3f\n       %3.3f\n",
		alpha[0], alpha[1], alpha[2] );
	
	struct timeval tv;
		struct timezone tz = {
		.tz_minuteswest = 0,
		.tz_dsttime = 0
	};
	gettimeofday( &tv, &tz );
}