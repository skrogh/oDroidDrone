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


class Imu {
private:
	// filedescriptors
	int gpioFd, spiFd;
	// Spi settings
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

	//
	void inline clearSpiInt( void );
	void gpioIntHandler( void );
	//
	void *imuThread( void *This );

public:



	Imu( const char *spiDevice, const char *gpioDevice );
	~Imu( void );
};

#endif//_IMU_H_