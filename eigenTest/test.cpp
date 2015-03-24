#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "odometry.hpp"


int main()
{
	Calib calib;
	calib.o_x = 300.8859;
	calib.o_y = 222.5206;
	calib.f_x = 411.1170;
	calib.f_y = 409.9516;
	calib.k1 = -0.3453;
	calib.k2 = 0.1012;
	calib.t1 = -0.0003;
	calib.t2 = 0.0014;
	calib.CI_q = Eigen::Quaternion<double>(
			0.000000000000000,
			-0.000000000000000,
			0.382683432365090,
			-0.923879532511287
	);
	calib.C_p_I = Eigen::Vector3d( 0.0, 0.0, -0.056 );
	calib.g = 9.82;
	calib.delta_t = 0.0025;
	calib.imageOffset = 0.033;
	calib.sigma_gc = 5.0e-04;
	calib.sigma_ac = 5.0e-04;
	calib.sigma_wgc = 0.05;
	calib.sigma_wac = 0.1;
	calib.sigma_Im = 40;
	calib.sigma_dc = 0.05;

	std::cout << "calib is:\n" << calib << std::endl;
	MSCKF msckf( &calib );

	double a_m[3] = { 0, 0, 9.82 };
	double g_m[3] = { 0, 0, 0 };

	std::cout << "msckf is:\n" << msckf << std::endl;

	for( int i = 0; i < 400; i++ ) {
		msckf.propagate( a_m, g_m );
	}

	std::cout << std::setprecision(3) << "msckf is:\n" << msckf << std::endl;
 
	std::cout << std::setprecision(3) << "Sigma is " << msckf.sigma.rows() << "x" << msckf.sigma.cols() << " :\n" << msckf.sigma << std::endl;
	std::cout << std::setprecision(3) << "State is " << msckf.x.rows() << "x" << msckf.x.cols() << " :\n" << msckf.x << std::endl;
	msckf.augmentState();
	std::cout << std::setprecision(3) << "Sigma is " << msckf.sigma.rows() << "x" << msckf.sigma.cols() << " :\n" << msckf.sigma << std::endl;
	std::cout << std::setprecision(3) << "State is " << msckf.x.rows() << "x" << msckf.x.cols() << " :\n" << msckf.x << std::endl;
}
