#include "imuFromFile.hpp"

// uses spidev f
ImuFF::ImuFF( char* fileName ) {
	file.open( fileName );
	ImuMeas_t element;
	int ping;
	std::string line;
	while (std::getline(file, line))
	{
		sscanf( line.c_str(), "%d.%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
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

ImuFF::~ImuFF( ) {
	file.close();
}

void ImuFF::fifoPush( const ImuMeas_t &element ) {
	dataFifo.push_back( element );
}

bool ImuFF::fifoPop( ImuMeas_t &element ) {
	if( !dataFifo.empty() ) {
		element = dataFifo.front( );
		dataFifo.pop_front( );
		return true;
	}
	return false;
}

bool ImuFF::fifoPeak( unsigned int n, ImuMeas_t &element ) {
	if( dataFifo.size() > n ) {
		element = dataFifo.at( n );
		return true;
	}
	return false;
}


unsigned int ImuFF::fifoSize( void ) {
	unsigned int size = dataFifo.size();
	return size;
}