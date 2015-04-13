#include "imuFromFile.hpp"

// uses spidev f
Imu::Imu( char* fileName ) {
	file.open( fileName );
	ImuMeas_t element;
	int ping;
	std::string line;
	while (std::getline(file, line))
	{
		sscanf( line.c_str(), "%d.%d, %e, %e, %e, %e, %e, %e, %e, %e, %e, %d",
			&(element.timeStamp.tv_sec), &(element.timeStamp.tv_usec),
			&(element.acc[0]), &(element.acc[1]), &(element.acc[2]),
			&(element.gyro[0]), &(element.gyro[1]), &(element.gyro[2]),
			&(element.alpha[0]), &(element.alpha[1]), &(element.alpha[2]),
			&ping
		);
		element.distValid = (ping != 0);
		element.dist = ping * ( 340.29 / 80000000.0 ) / 2.0 - 0.0565;
		this->fifoPush( element );
	}
}

Imu::~Imu( ) {
	file.close();
}

void Imu::fifoPush( const ImuMeas_t &element ) {
	dataFifo.push_back( element );
}

bool Imu::fifoPop( ImuMeas_t &element ) {
	if( !dataFifo.empty() ) {
		element = dataFifo.front( );
		dataFifo.pop_front( );
		return true;
	}
	return false;
}

bool Imu::fifoPeak( unsigned int n, ImuMeas_t &element ) {
	if( dataFifo.size() > n ) {
		element = dataFifo.at( n );
		return true;
	}
	return false;
}


unsigned int Imu::fifoSize( void ) {
	unsigned int size = dataFifo.size();
	return size;
}