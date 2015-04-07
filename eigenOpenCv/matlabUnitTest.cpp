#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/time.h>

#include "feature.hpp"
#include "odometry.hpp"
#include "common.hpp"
#include "imu.hpp"

int main( int argc, char** argv )
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
	calib.CI_q = Eigen::QuaternionAlias<double>(
			0.000000000000000, // note that the real value comes first in the eigen quaterneon constructor
			-0.000000000000000,
			0.382683432365090,
			-0.923879532511287
	);
	calib.C_p_I = Eigen::Vector3d( 0.0, 0.0, -0.056 );
	calib.g = 9.82;
	calib.delta_t = 0.0025;
	calib.imageOffset.tv_sec = 0;
	calib.imageOffset.tv_usec = 33000;
	calib.sigma_gc = 0.001;//5.0e-04;
	calib.sigma_ac = 0.008;//5.0e-04;
	calib.sigma_wgc = 0.0001;
	calib.sigma_wac = 0.0001;
	calib.sigma_Im = 40;
	calib.sigma_hc = 0.05;
	calib.minFrame = 1;
	std::cout << "calib is:\n" << calib << std::endl;


	MSCKF msckf( &calib );
	// initialize upside down
	msckf.x.block<4,1>(0,0) << 0,1,0,0;

	// Test triangulate
	// set position and push some frames:
	msckf.x.block<3,1>(4,0) << 0,0,1;
	msckf.augmentState();
	msckf.x.block<3,1>(4,0) << 0,0,0;
	msckf.augmentState();
	msckf.x.block<3,1>(4,0) << 0,0,0;
	Eigen::MatrixX2d z( 1, 2 );
	z << 320,240;
	std::cout << "Triangulate test1: " << msckf.triangulate( z ) << std::endl;
	z << 0,0;
	std::cout << "Triangulate test2: " << msckf.triangulate( z ) << std::endl;
	z << 640,0;
	std::cout << "Triangulate test3: " << msckf.triangulate( z ) << std::endl;
	z << 0,480;
	std::cout << "Triangulate test4: " << msckf.triangulate( z ) << std::endl;
	z << 640,480;
	std::cout << "Triangulate test5: " << msckf.triangulate( z ) << std::endl;



}