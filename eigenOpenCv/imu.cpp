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
	pthread_mutex_init( &fifoMutex, NULL );

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
	pthread_attr_t attr;
	pthread_attr_init(&attr);

	struct sched_param param;
	int policy = SCHED_FIFO;
	param.sched_priority = sched_get_priority_max( policy );

	pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED );
	pthread_attr_setschedpolicy( &attr, policy );
	pthread_attr_setschedparam( &attr, &param );

	ret = pthread_create( &thread, &attr, &Imu::imuThreadWrapper, this );
	if (ret == -1) {
		perror("Creating thread");
	}

	pthread_attr_destroy(&attr);
}

Imu::~Imu( ) {
	printf( "Signaling to end imu thread and join\n" );
	endThread = true;
	pthread_join( thread, NULL );
	printf( "Thread joined\n" );
	close(gpioFd);
	close(spiFd);
}

char inline Imu::clearSpiInt( void ) {
	// clear interrupt
	char c;
	lseek ( gpioFd, 0, SEEK_SET);
	read( gpioFd, &c, 1);
	return c;
}

void* Imu::imuThread( void ) {
	int rc; // return code

	// Poll struct
	struct pollfd fdset = {};
	fdset.fd = gpioFd;
	fdset.events = POLLPRI;

	// Timing struct
	struct timeval tv;
	struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;

	this->clearSpiInt();

	while( !endThread ) {
		fdset.revents = 0;

		// start waiting for next interrupt
		rc = poll( &fdset, 1, timeout );
		// clear interrupt, read status
		char value = this->clearSpiInt();
		// Get time
		gettimeofday( &tv, &tz );

		// Check if interrupt request failed
		if (rc < 0) {
			perror("poll() failed!");
		}

		// Check if timed out
		if (rc == 0){
			// check if interrupt is high, if it is we previously missed a read/write so take it now
			if ( value == '1' ) {
				this->gpioIntHandler( tv );
				continue;
			}
		}

		// Check if correct interrupt
		if (fdset.revents & POLLPRI) {
			this->gpioIntHandler( tv );
		}
	}
	return (NULL);
}

void Imu::gpioIntHandler( const struct timeval& tv ) {
	int ret;
	uint8_t tx[MESSAGE_LENGTH] = { 0 };
	uint8_t rx[MESSAGE_LENGTH] = { 0 };
	struct spi_ioc_transfer tr = {};
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = MESSAGE_LENGTH;
		tr.delay_usecs = delay;
		tr.speed_hz = speed;
		tr.bits_per_word = bits;


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
	double dist = ping * ( 340.29 / 80000000.0 ) / 2.0 - 0.00678;

	ImuMeas_t element = {};
	element.timeStamp = tv;
	element.dist = dist;
	element.distValid = ( ping != 0 );
	for ( int i = 0; i < 3; i++ ) {
		element.acc[i] = acc[i];
		element.gyro[i] = gyro[i];
		element.alpha[i] = alpha[i];
	}
	this->fifoPush( element );
}

void Imu::fifoPush( const ImuMeas_t &element ) {
	pthread_mutex_lock( &fifoMutex );
	dataFifo.push_back( element );
	pthread_mutex_unlock( &fifoMutex );
}

bool Imu::fifoPop( ImuMeas_t &element ) {
	pthread_mutex_lock( &fifoMutex );
	if( !dataFifo.empty() ) {
		element = dataFifo.front( );
		dataFifo.pop_front( );
		pthread_mutex_unlock( &fifoMutex );
		return true;
	}
	pthread_mutex_unlock( &fifoMutex );
	return false;
}

bool Imu::fifoPeak( unsigned int n, ImuMeas_t &element ) {
	pthread_mutex_unlock( &fifoMutex );
	if( dataFifo.size() > n ) {
		element = dataFifo.at( n );
		pthread_mutex_unlock( &fifoMutex );
		return true;
	}
	pthread_mutex_unlock( &fifoMutex );
	return false;
}


unsigned int Imu::fifoSize( void ) {
	pthread_mutex_unlock( &fifoMutex );
	unsigned int size = dataFifo.size();
	pthread_mutex_unlock( &fifoMutex );
	return size;
}
