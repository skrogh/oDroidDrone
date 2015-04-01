#include <stdio.h>
#include <iostream>
#include "imu.hpp"


/** @function main */
int main( int argc, char** argv )
{
	Imu imu( "/dev/spidev1.0", "/sys/class/gpio/gpio199/value" );

	char wait;
	std::cout << "Press a key..." << std::endl;
	std::cin >> wait;
	std::cout << "Progam ended" << std::endl;
	return 0;
}
