#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/time.h>

#include "feature.hpp"
#include "odometry.hpp"
#include "common.hpp"
#include "imu.hpp"
#include "quaternionAlias.hpp"

int main( int argc, char** argv )
{

	QuaternionAlias<double> q_a1( 1, 0, 0, 0 );
	QuaternionAlias<double> q_a2 = QuaternionAlias<double>( 1, 0, 0, 0 );
	Quaternion<double> q1 = Quaternion<double>( 1, 0, 0, 1 );
	Quaternion<double> q2 = Quaternion<double>( 1, 0, 0, 0 );

	std::cout << "q_a1: " << q_a1.coeffs() << " q_a2: " << q_a2.coeffs() << std::endl;
	std::cout << "q_a1 * q_a2: " << (q_a1*q_a2).coeffs() << std::endl;
	std::cout << "q1: " << q1.coeffs() << " q2: " << q2.coeffs() << std::endl;
	std::cout << "q1 * q2: " << (q1*q2).coeffs() << std::endl;

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
	calib.imageOffset.tv_sec = 0;
	calib.imageOffset.tv_usec = 33000;
	calib.sigma_gc = 0.001;//5.0e-04;
	calib.sigma_ac = 0.008;//5.0e-04;
	calib.sigma_wgc = 0.0001;
	calib.sigma_wac = 0.0001;
	calib.sigma_Im = 10;
	calib.sigma_hc = 0.05;
	calib.minFrame = 1;
	std::cout << "calib is:\n" << calib << std::endl;


	MSCKF msckf( &calib );
	// Start upside down
	msckf.x.block<4,1>(0,0) << 0, 1, 0, 0;
	// Start 10cm off the ground
	msckf.x.block<3,1>(4,0) << 0, 0, 0.1;
	//acc offset
	msckf.x.block<3,1>(4+3+3+3,0) << 0, 0, 0;

	// Set initial uncertancy
	msckf.sigma.diagonal().block<3,1>(0,0) << 0.05, 0.05, 0.05;
	msckf.sigma.diagonal().block<3,1>(3,0) << 0, 0, 0.1;
	msckf.sigma.diagonal().block<3,1>(6,0) << 0, 0, 0;
	msckf.sigma.diagonal().block<3,1>(9,0) << 0.01, 0.01, 0.01;
	msckf.sigma.diagonal().block<3,1>(12,0) << 0.1, 0.1, 0.1;


	std::ofstream logFile;
 	logFile.open ("log.csv");


	cv::VideoCapture cap(0);
	cv::Mat image;
	cv::namedWindow( "DebugDraw" );

	CameraMeasurements cameraMeasurements;
	CameraDetector cameraDetector = CameraDetector( );

	Imu imu( "/dev/spidev1.0", "/sys/class/gpio/gpio199/value" );

	// Wait for flightcontroller to start
	{
		std::cout << "Waiting for flightcontroller to boot" << std::endl;
		ImuMeas_t element;
		while( !imu.fifoPop( element ) );
		std::cout << "Flightcontroller booted" << std::endl;
	}

	struct timeval tv;
	struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;
	
	//
	// Spool up image display
	//
	cap.grab();
	cap.retrieve( image );
	cv::imshow("DebugDraw", image );
	cv::waitKey(1);

	//
	// Clear Imu buffer
	//
	{
		ImuMeas_t element;
		while( imu.fifoPop( element ) );
	}
	int n = 0;
	int nn = 0;
	while( cv::waitKey(1) != 27 ) {
		//cap.grab();
		//cap.grab();
		//cap.grab();
		//cap.grab();
		//cap.grab();
		gettimeofday( &tv, &tz );
		//cap.retrieve( image );
		msckf.debugImg = image.clone();

		//
		// Propagate up to new image ( can be run in parallel with feature detection)
		//
		while( 1 ) {
			ImuMeas_t element;
			// Wait for at least one imu measurement
			while( !imu.fifoPop( element ) );

			// Propagate
			msckf.propagate( element.acc, element.gyro );
			logFile << msckf.x.block<16,1>(0,0).transpose() << "\t";
			logFile << msckf.sigma.diagonal().block<15,1>(0,0).transpose() << "\n";

			// If valid distance measurement, update with that
			if ( element.distValid ) {
				if ( n > 5 ) {
					msckf.updateHeight( element.dist );
					n = 0;
				} else {
					n++;
				}
			}

			// Get time of image without delay
			struct timeval imageTime;
			timersub( &tv, &(calib.imageOffset), &imageTime );

			// If image is older that propagated point, update
			if ( timercmp( &imageTime, &(element.timeStamp), < ) ) {
				timersub( &(element.timeStamp), &imageTime, &imageTime );
				std::cout << "Image/IMU time difference: " <<
				imageTime.tv_sec << "." << std::setfill('0') << std::setw(6) << imageTime.tv_usec << "s" << std::setfill(' ') << std::endl;
				break;
			}
		}

		//
		// Detect features ( can be run in parallel with propagation)
		//
		if( nn++ < 2 )
			cameraDetector.detectFeatures( image, cameraMeasurements );
		cameraDetector.addFeatures( cameraMeasurements );

		//
		// We have propagated and got a new image, time to update with camera data
		//
		msckf.updateCamera( cameraMeasurements );

				// Iterate over meas and draw all non lost elements:
		for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ++meas_j ) {
			if ( !meas_j->isLost ) {
				Eigen::MatrixX2d& z = meas_j->z;
				cv::Point pt = Point( z( z.rows()-1, 0 ), z( z.rows()-1, 1 ) );
				cv::circle( msckf.debugImg, pt, 4, Scalar( 255, 0, 0 ) );
				for( int i = 0; i < z.rows() - 1; i++ ) {
					cv::Point pt1 = Point( z( i, 0 ), z( i, 1 ) );
					cv::Point pt2 = Point( z( i+1, 0 ), z( i+1, 1 ) );
					cv::line( msckf.debugImg, pt1, pt2, Scalar( 255, 0, 0 ) );
				}

			}
		}

		cv::imshow("DebugDraw", msckf.debugImg );

		//
		// Print state
		//
		std::cout << "msckf is:\n" << msckf << std::endl;
		std::cout << "sigma is:" << msckf.sigma.rows() << "x" << msckf.sigma.cols() << std::endl;
	}
	logFile.close();
}