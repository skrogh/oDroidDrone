#include "telemetry.hpp"
#include <iostream>
#include <errno.h>

void Telemetry::error( const char *msg )
{
	perror(msg);
	endThread = true;
}


Telemetry::Telemetry( int portno_ )
{
	portno = portno_;

	requestSend = false;
	endThread = false;

	countInBuffer = 0;

	// setup mutex
	pthread_mutex_init( &bufferMutex, NULL );

	//
	// start thread
	//
	int ret = pthread_create( &thread, NULL, &Telemetry::telemetryThreadWrapper, this );
	if (ret == -1) {
		perror("Creating thread");
	}
}

Telemetry::~Telemetry( )
{
	std::cout << "Requesting to stop telemetry server" << std::endl;
	endThread = true;
	pthread_join( thread, NULL );
	std::cout << "Telemetry server stopped" << std::endl;
}


void* Telemetry::telemetryThread( void )
{
	std::cout << "Telemetry server: Started" << std::endl;

	// 
	// Create listing socket
	int sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	if (sockfd < 0) 
		error("ERROR opening socket");

	struct sockaddr_in serv_addr = {0};
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);

	// Bind listing socket to port
	if ( bind( sockfd, (struct sockaddr *) &serv_addr,
			sizeof( serv_addr) ) < 0 ) 
		error("ERROR on binding");

	while( !endThread ) {
		// Start listening for connections
		listen( sockfd, 1 );
		struct sockaddr_in cli_addr;
		socklen_t clilen = sizeof(cli_addr);

		// Block and wait for connection
		newsockfd = accept(sockfd, 
				(struct sockaddr *) &cli_addr, 
				&clilen);
		if (newsockfd < 0) 
				error("ERROR on accept");

		std::cout << "Telemetry server: Connected" << std::endl;

		// send data
		while( !endThread ) {
			//If not already signaled to send, wait for signal
			if ( !requestSend )
			{
				std::unique_lock<std::mutex> lck( requestSendSignalMtx );
				requestSendSignal.wait( lck );
				requestSend = false;
			}

			pthread_mutex_lock( &bufferMutex );
			int n = ::send( newsockfd, buffer, countInBuffer, MSG_NOSIGNAL );
			countInBuffer = 0;
			pthread_mutex_unlock( &bufferMutex );
			if ( n < 0 ) {
				if ( n = EPIPE )
				{
					std::cout << "Telemetry server: Disconnected" << std::endl;
				}
				else
				{
					error( "ERROR writing to socket" );
				}
				break;
			} else if ( n == 0 ) {
				std::cout << "Telemetry server: Disconnected" << std::endl;
				break;
			}

			/*
			char buffer[256] = {0};
			int n = read( newsockfd, buffer, 255 );
			
			else
				printf( "Here is the message: %s\n", buffer );
				*/
		}
		std::cout << "Telemetry server: Closing socket" << std::endl;
		close(newsockfd);
	}
	close(sockfd);
	std::cout << "Telemetry server: Ended" << std::endl;
	return (NULL);
}

bool Telemetry::send( const void *buf, size_t count ) {
	if( count > TELEMETRY_BUFFER_SIZE )
		return true;
	pthread_mutex_lock( &bufferMutex );
	memcpy ( buffer, buf, count );
	countInBuffer = count;
	pthread_mutex_unlock( &bufferMutex );
	requestSend = true;
	std::unique_lock<std::mutex> lck( requestSendSignalMtx );
	requestSendSignal.notify_all( );

	return false;
}