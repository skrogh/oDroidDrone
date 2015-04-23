#include "telemetry.hpp"
#include <iostream>


static void error( const char *msg )
{
	perror(msg);
}


Telemetry::Telemetry( int portno_ )
{
	endThread = false;
	portno = portno_;
	//
	// start thread
	//
	int ret = pthread_create( &thread, NULL, &Telemetry::TelemetryThreadWrapper, this );
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

	// Print all recieved data
	while( !endThread ) {
		char buffer[256] = {0};
		int n = read( newsockfd, buffer, 255 );
		if ( n < 0 )
			error( "ERROR reading from socket" );
		printf( "Here is the message: %s\n", buffer );
	}
	close(newsockfd);
	close(sockfd);
	std::cout << "Telemetry server: Ended" << std::endl;
	return (NULL);
}