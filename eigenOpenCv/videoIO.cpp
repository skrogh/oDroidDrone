#include "videoIO.hpp"

VideoIn::VideoIn( int dev )
{
	// open capture device
	cap = cv::VideoCapture(dev);
	cap.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

	// setup mutex
	pthread_mutex_init( &capMutex, NULL );
	requestingImage = false;
	endThread = false;

	//
	// start thread
	//
	pthread_attr_t attr;
	pthread_attr_init(&attr);

	struct sched_param param;
	int policy = SCHED_FIFO;
	param.sched_priority = sched_get_priority_min( policy ); // make it realtime, but not hard realtime

	pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED );
	pthread_attr_setschedpolicy( &attr, policy );
	pthread_attr_setschedparam( &attr, &param );

	int ret = pthread_create( &thread, &attr, &VideoIn::videoThreadWrapper, this );
	if (ret == -1) {
		perror("Creating thread");
	}

	pthread_attr_destroy(&attr);
}
VideoIn::~VideoIn( )
{
	endThread = true;
	pthread_join( thread, NULL );
}


void* VideoIn::videoThread( void )
{
	while( !endThread ) {
		// construct timezone struct
		struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;

		// If an image was requested, make sure the requester gets time
		if ( requestingImage )
		{
			std::unique_lock<std::mutex> lck( doneRequestingImageMtx );
			doneRequestingImage.wait( lck );
		}
		pthread_mutex_lock( &capMutex );
		cap.grab();
		gettimeofday( &timeStamp, &tz );
		pthread_mutex_unlock( &capMutex );
	}
}

void VideoIn::requestImage( cv::Mat &image, timeval& tv )
{
	requestingImage = true;
	pthread_mutex_lock( &capMutex );
	cap.retrieve( image );
	tv = timeStamp;
	pthread_mutex_unlock( &capMutex );
	std::unique_lock<std::mutex> lck( doneRequestingImageMtx );
	doneRequestingImage.notify_all( );
}