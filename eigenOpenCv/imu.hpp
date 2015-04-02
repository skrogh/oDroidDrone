#ifndef _IMU_H_
#define _IMU_H_

#include <pthread.h>
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
#include <deque>


typedef struct{
	double acc[3];
	double gyro[3];
	double alpha[3];
	double dist;
	struct timeval timeStamp;
} ImuMeas_t;

class Imu {
private:
	// filedescriptors
	int gpioFd, spiFd;
	//
	// Spi settings
	//
	uint32_t mode;
	uint32_t bits;
	uint32_t speed;
	uint16_t delay;
	uint32_t timeout;

	//
	// Thread handing
	//
	pthread_t thread;
	bool volatile endThread;
	pthread_mutex_t fifoMutex;

	//
	// Data FIFO
	//
	std::deque<ImuMeas_t> dataFifo;

	//
	// Functions
	//

	// Clear SPI interrupt
	void inline clearSpiInt( void );
	// Interrupt handler
	void gpioIntHandler( const struct timeval& tv );
	// Interrupt thread
	void* imuThread( void );
	static void * imuThreadWrapper( void *This )
			{ return ( ( (Imu *) This )->imuThread() ); }

public:
	Imu( const char *spiDevice, const char *gpioDevice );
	~Imu( void );

	// Push element into the fifo
	void fifoPush( const ImuMeas_t &element );
	// Pop ad copy to element.
	// Retruns true if there was something to pop
	bool fifoPop( ImuMeas_t &element );
	// Peak at the n'th oldest element and copy it to element.
	// Retruns true, if there was an element there.
	bool fifoPeak( unsigned int n, ImuMeas_t &element );
	// Get size of fifo
	unsigned int fifoSize( void );
};

#endif//_IMU_H_