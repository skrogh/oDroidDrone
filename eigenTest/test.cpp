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

	Matrix3d m1;
	MatrixXd m2;
	MatrixXd m3;

	m1 << 1, 2, 3,
	      4, 5, 6,
	      7, 8, 9;
	m3 << m1, m1;
	m2 << m1,
	      m1;

	std::cout << m1 << std::endl;
	std::cout << m2 << std::endl;
	std::cout << m3 << std::endl;
}
