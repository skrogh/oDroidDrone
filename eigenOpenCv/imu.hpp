#ifndef _IMU_H_
#define _IMU_H_

#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <atomic>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <endian.h>
#include <deque>
#include <mutex>
#include <condition_variable>


typedef struct{
	double acc[3];
	double gyro[3];
	double alpha[3];
	double dist;
	bool distValid;
	struct timeval timeStamp;
} ImuMeas_t;

typedef struct{
	float x;
	float y;
	float yaw;
	float z;
} FlightControllerOut_t;

class ImuFifo {
protected:
	//
	// Data FIFO
	//
	pthread_mutex_t fifoMutex;
	std::deque<ImuMeas_t> dataFifo;
	//
	// Signaling
	//
	std::condition_variable notEmpty;
	std::mutex notEmptyMtx;


public:
	// Constructor
	ImuFifo( void );
	// Push element into the fifo
	void fifoPush( const ImuMeas_t &element );
	// Pop ad copy to element.
	// Retruns true if there was something to pop
	bool fifoPop( ImuMeas_t &element );
	// Peak at the n'th oldest element and copy it to element.
	// Retruns true, if there was an element there.
	bool fifoPeak( unsigned int n, ImuMeas_t &element );
	//
	//
	void fifoCloneTo( ImuFifo& dest );
	// Get size of fifo
	unsigned int fifoSize( void );
	// Wait for not empty (doesn't hog cpu)
	void waitNotEmpty( void );
};

class Imu: public ImuFifo {
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
	std::atomic<bool> endThread;

	//
	// Output to flight controller
	//
	FlightControllerOut_t flightControllerOut;
	pthread_mutex_t flightControllerOutMtx;

	//
	// Functions
	//

	// Clear SPI interrupt
	char inline clearSpiInt( void );
	// Interrupt handler
	void gpioIntHandler( const struct timeval& tv );
	// Interrupt thread
	void* imuThread( void );
	static void * imuThreadWrapper( void *This )
			{ return ( ( (Imu *) This )->imuThread() ); }

public:
	Imu( const char *spiDevice, const char *gpioDevice );
	~Imu( void );
	void setOutput( float x, float y, float yaw, float z );
};

#endif//_IMU_H_
