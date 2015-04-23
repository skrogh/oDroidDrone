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

#define TELEMETRY_BUFFER_SIZE (256)


class Telemetry
{
	//
	// Thread handing
	//
	pthread_t thread;
	std::atomic<bool> endThread;
	pthread_mutex_t bufferMutex;

	std::atomic<bool> requestSend;
	std::condition_variable requestSendSignal;
	std::mutex requestSendSignalMtx;

	//
	// Socket handling
	//
	int newsockfd;
	int portno;
	uint8_t buffer[TELEMETRY_BUFFER_SIZE];
	int countInBuffer;
	


	void* telemetryThread( void );
	static void * telemetryThreadWrapper( void *This )
			{ return ( ( (Telemetry *) This )->telemetryThread() ); }
	void error( const char *msg );


public:
	Telemetry( int portno_ );
	~Telemetry( void );

	// Sends a message, returns true on error (message too long)
	bool send( const void *buf, size_t count );

};


#endif//_VIDEOIO_H_