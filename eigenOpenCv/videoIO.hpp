#ifndef _VIDEOIO_H_
#define _VIDEOIO_H_

#include <pthread.h>
#include <condition_variable>
#include <stdint.h>
#include <sys/time.h>
#include <atomic> 
#include "opencv2/opencv.hpp"


class VideoIn
{
	//
	// Thread handing
	//
	pthread_t thread;
	std::atomic<bool> endThread;
	pthread_mutex_t capMutex;
	std::atomic<bool> requestingImage;
	std::condition_variable doneRequestingImage;

	//
	// Capture device and timing
	//
	cv::VideoCapture cap;
	timeval timeStamp;

	void* videoThread( void );
	static void * videoThreadWrapper( void *This )
			{ return ( ( (Imu *) This )->videoThread() ); }



public:
	VideoIn( int dev = 0 );
	~VideoIn( void);
	void requestImage( cv::Mat &image, timeval& tv );

};


#endif//_VIDEOIO_H_