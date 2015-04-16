#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <ctype.h>
#include <random>
#include "lkTracker.hpp"
#include "odometry.hpp"
#include "imu.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;

Vector2d featureUndistort( const Vector2d &src, const Calib *calib, unsigned int itterations = 3 )
{
	Vector2d beta( 
		( src( 0 ) - calib->o_x ) / calib->f_x,
		( src( 1 ) - calib->o_y ) / calib->f_y
	);

	// itterate:
	while( itterations-- ) {
		// Residual
		Matrix<double,2,1> r = src - cameraProject( beta(0), beta(1), 1, calib );
		// Jacobian
		Matrix<double,2,2> Jf = jacobianH( beta(0), beta(1), 1, calib ).block<2,2>(0,0);
		// New estimate
		beta = beta + (Jf.transpose()*Jf).inverse() * Jf.transpose() * r;
	}
	return Matrix<double,2,1>( beta(0), beta(1) );
}

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
			 0.000000000000000,
			-0.707106781186548,
			 0.707106781186548,
			 0.000000000000000
	);
	calib.C_p_I = Eigen::Vector3d( 0.0, -0.044, -0.037 );
	calib.g = 9.82;
	calib.delta_t = 0.0025;
	calib.imageOffset.tv_sec = 0;
	calib.imageOffset.tv_usec = 33000*5; // delay of 5 images
	calib.sigma_gc = 0.001;//5.0e-04;
	calib.sigma_ac = 0.008;//5.0e-04;
	calib.sigma_wgc = 0.0001;
	calib.sigma_wac = 0.0001;
	calib.sigma_Im = 40;
	calib.sigma_hc = 0.05;
	calib.minFrame = 1;
	std::cout << "calib is:\n" << calib << std::endl;


	MSCKF msckf( &calib );
	// Start upside down
	msckf.x.block<4,1>(0,0) << 0, 0, 0, 1; // upright
	// Start 10cm off the ground
	msckf.x.block<3,1>(4,0) << 0, 0, 0.5; // 50cm from ground
	//acc offset
	msckf.x.block<3,1>(4+3+3+3,0) << 0, 0, 0;



	VideoCapture cap;
	cap.open(0);

	// size
	cap.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

	namedWindow( "Features", 1 );

	Imu imu( "/dev/spidev1.0", "/sys/class/gpio/gpio199/value" );
	struct timeval tv;
	struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;
	//
	// Clear Imu buffer
	//
	gettimeofday( &tv, &tz );
	{
		ImuMeas_t element;
		while( imu.fifoPop( element ) )
		{
			// Get time of image without delay
			struct timeval imageTime;
			timersub( &tv, &(calib.imageOffset), &imageTime );

			// If image is older that now
			if ( timercmp( &imageTime, &(element.timeStamp), < ) )
			{
				timersub( &(element.timeStamp), &imageTime, &imageTime );
				break;
			}
		}
	}

	Mat gray, prevGray;
	LKTracker tracker;
	double pX=0, pY=0;

	int ignoredHeights = 0;
	for(;;)
	{
		Mat frame;
		cap.grab();
		gettimeofday( &tv, &tz );
		cap.retrieve(frame);

		cvtColor(frame, gray, COLOR_BGR2GRAY);
		if(prevGray.empty())
		{
			gray.copyTo(prevGray);
			// skip first image
			msckf.augmentState( );
			continue;
		}

		//
		// Propagate up to new image ( can be run in parallel with feature detection)
		//
		while( 1 ) {
			ImuMeas_t element;
			// Wait for at least one imu measurement
			while( !imu.fifoPop( element ) );

			// Propagate
			msckf.propagate( element.acc, element.gyro );

			// If valid distance measurement, update with that
			if ( element.distValid )
			{
				if ( ignoredHeights >= 0 )
				{
					msckf.updateHeight( element.dist );
					ignoredHeights = 0;
				}
				else
				{
					ignoredHeights++;
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
		// Update from camera
		//

		tracker.detectFeatures( gray, prevGray );

		//
		// undistort points
		//

		// Initialize
		Matrix2Xd points(2, tracker.points.size());
		Matrix2Xd prevPoints(2, tracker.points.size());

		// Undistort
		for ( int i = 0; i < points.cols(); i++ )
		{
			points.col(i) = featureUndistort( Vector2d( tracker.points[i].x, tracker.points[i].y ), msckf.calib);
			prevPoints.col(i) = featureUndistort( Vector2d( tracker.prevPoints[i].x, tracker.prevPoints[i].y ), msckf.calib);
		}

		//
		// Project on ground
		//
		for ( int i = 0; i < points.cols(); i++ )
		{
			const Calib* calib = msckf.calib;
			const VectorXd &x = msckf.x;
			// Calculate camera state
			QuaternionAlias<double> IG_q( x.block<4,1>( 0, 0 ) );
			QuaternionAlias<double> CG_q = calib->CI_q * IG_q;
			Vector3d G_p_I = x.block<3,1>( 4, 0 );
			// force x and y to 0 to get points relative to position
			G_p_I(0) = 0;
			G_p_I(1) = 0;
			Vector3d G_p_C = G_p_I - CG_q.conjugate()._transformVector( calib->C_p_I );

			// Calculate feature position estimate
			Vector3d C_theta_i( points(0,i), points(1,i), 1 );
			Vector3d G_theta_i = CG_q.conjugate()._transformVector( C_theta_i );
			double t_i = - G_p_C( 2 ) / G_theta_i( 2 );
			points.col( i ) = ( t_i * G_theta_i + G_p_C ).block<2,1>(0,0);

			// Calculate previous camera state
			QuaternionAlias<double> IpG_q( x.block<4,1>( ODO_STATE_SIZE + 0, 0 ) );
			QuaternionAlias<double> CpG_q = calib->CI_q * IpG_q;
			Vector3d G_p_Ip = x.block<3,1>( ODO_STATE_SIZE + 4, 0 );
			// force x and y to 0 to get points relative to position
			G_p_Ip(0) = 0;
			G_p_Ip(1) = 0;
			Vector3d G_p_Cp = G_p_Ip - CpG_q.conjugate()._transformVector( calib->C_p_I );

			// Calculate feature position estimate
			Vector3d Cp_theta_i( prevPoints(0,i), prevPoints(1,i), 1 );
			Vector3d Gp_theta_i = CpG_q.conjugate()._transformVector( Cp_theta_i );
			double t_pi = - G_p_Cp( 2 ) / Gp_theta_i( 2 );
			prevPoints.col( i ) = ( t_pi * Gp_theta_i + G_p_Cp ).block<2,1>(0,0);
		}

		//
		// Find geometric transform
		//

		// construct constraints
		Matrix<double, Dynamic, 5> C( points.cols()*2, 5 );
		for ( int i = 0; i < points.cols(); i++ )
		{
			const double &x = points(0,i);
			const double &y = points(1,i);
			const double &x_ = prevPoints(0,i);
			const double &y_ = prevPoints(1,i);
			C.row( i*2 )    << -y,  x,  0, -1,  y_;
			C.row( i*2 + 1) <<  x,  y,  1,  0, -x_;
		}


		if ( points.cols() >= 3 ) // only if 3 or more points (2 is needed, 1 extra for redundancy)
		{
			JacobiSVD<MatrixXd> svd( C, ComputeThinV );
			VectorXd V = svd.matrixV().rightCols<1>();
			// find transformation
			VectorXd h = V.head<4>() / V(4);
			// remove scaling
			h.block<2,1>(0,0).normalize();

			pX += h(2);
			pY += h(3);


			msckf.x.block<2,1>(4,0) += Vector2d( h(2), h(3) );
			double dAngle = -atan2( h(1), h(0) );
			msckf.x.block<4,1>(0,0) = 
				( QuaternionAlias<double>( cos(dAngle/2), 0, 0, sin(dAngle/2) )
				* QuaternionAlias<double>( msckf.x.block<4,1>(0,0) ) ).normalized().coeffs();

			cout << endl;
			cout << endl;
			cout << "n Points: " << points.cols() << endl;
			cout << "Moved: " << h(2) << ", " << h(3) << endl;
			cout << "Total: " << pX << ", " << pY << endl;
		}

		//
		// Calculate kalman gain
		//

		// calculate residual


		// update state fifo
		msckf.removeOldStates( 1 );
		msckf.augmentState( );


		for( int i = 0; i < tracker.points.size(); i++ )
		{
			line( frame, tracker.points[i], tracker.prevPoints[i], Scalar(0,255,0) );
			circle( frame, tracker.points[i], 2, Scalar(0,255,0) );
		}
		imshow("Features", frame);

		char c = (char)waitKey(1);
		if( c == 27 )
			break;
		switch( c )
		{
		case 'c':
			tracker.prevPoints.clear();
			tracker.points.clear();
			pX = 0;
			pY = 0;
			// Start upside down
			msckf.x.block<4,1>(0,0) << 0, 0, 0, 1; // upright
			// Start 10cm off the ground
			msckf.x.block<3,1>(4,0) << 0, 0, 0.5; // 50cm from ground
			break;
		}
		cv::swap(prevGray, gray);
	}
}