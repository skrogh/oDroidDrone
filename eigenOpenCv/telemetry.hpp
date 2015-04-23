#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <stdint.h>
#include <atomic> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>


class Telemetry
{
	//
	// Thread handing
	//
	pthread_t thread;
	std::atomic<bool> endThread;
	std::condition_variable doneRequestingImage;
	std::mutex doneRequestingImageMtx;

	//
	// Socket handling
	//
	int sockfd, newsockfd, portno;
	char buffer[256];
	int portno;
	


	void* telemetryThread( void );
	static void * telemetryThreadWrapper( void *This )
			{ return ( ( (Telemetry *) This )->telemetryThread() ); }



public:
	Telemetry( int portno_ );
	~Telemetry( void );

};


#endif//_VIDEOIO_H_