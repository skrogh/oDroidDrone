#include <stdio.h>
#include <iostream>
#include <iomanip>
#include "imu.hpp"


/** @function main */
int main( int argc, char** argv )
{
	Imu imu( "/dev/spidev1.0", "/sys/class/gpio/gpio199/value" );

	char wait;
	std::cout << "Press a key..." << std::endl;
	std::cin >> wait;
	std::cout << "Got: " << imu.fifoSize() << " Imu samples" << std::endl;

	std::cout
	<< std::fixed
	<< std::setprecision(2);
	ImuMeas_t element;
	double timePrev = 0;
	while( imu.fifoPop( element ) ) {
		double timeNow = element.timeStamp.tv_sec + element.timeStamp.tv_usec/1000000;
		std::cout <<
		"Time: " <<
		element.timeStamp.tv_sec << "." << std::setfill('0') << std::setw(6) << element.timeStamp.tv_usec << "s\n" <<
		"Delta time: " <<
		timeNow - timePrev << "s\n" <<
		"Acc: " <<
		element.acc[0] << ", " << element.acc[1] << ", " <<  element.acc[2] << "\n" << 
		"Gyro: " <<
		element.gyro[0] << ", " << element.gyro[1] << ", " <<  element.gyro[2] << "\n" << 
		"Alpha: " <<
		element.alpha[0] << ", " << element.alpha[1] << ", " <<  element.alpha[2] << "\n" << 
		"Dist: " <<
		element.dist << "\n" << std::endl;
		timePrev = timneNow;
	}
	std::cout << "Progam ended" << std::endl;
	return 0;
}
