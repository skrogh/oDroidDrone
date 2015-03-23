#include <iostream>
#include <Eigen/Dense>
#include "odometry.hpp"


int main()
{
	Calib calib;
	calib.o_x = 10;
	calib.o_y = 10;
	std::out << calib << std::endl;
	MSCKF msckf( calib );
}
