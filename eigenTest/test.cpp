#include <iostream>
#include <Eigen/Dense>
#include "odometry.hpp"


int main()
{
	Calib calib;
	calib.o_x = 10;
	calib.o_y = 10;
	std::cout << "calib is:\n" << calib << std::endl;
	MSCKF msckf( &calib );

	double a_m[3] = { 0, 0, 1 };
	double g_m[3] = { 0, 0, 0 };

	std::cout << "msckf is:\n" << msckf << std::endl;
	msckf.propagateState( a_m, g_m );
	std::cout << "msckf is:\n" << msckf << std::endl;
}
