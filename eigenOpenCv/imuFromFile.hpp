#ifndef _IMUFF_H_
#define _IMUFF_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include <deque>
#include <fstream>


typedef struct{
	double acc[3];
	double gyro[3];
	double alpha[3];
	double dist;
	bool distValid;
	struct timeval timeStamp;
} ImuMeas_t;

class Imu {
private:
	// filesdescriptors
	std::ifstream file;

	//
	// Data FIFO
	//
	std::deque<ImuMeas_t> dataFifo;



public:
	Imu( char* fileName );
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