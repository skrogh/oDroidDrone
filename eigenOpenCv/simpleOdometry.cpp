#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ctype.h>
#include <random>
#include "lkTracker.hpp"
#include "odometry.hpp"

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
	// Start upside down
	msckf.x.block<4,1>(0,0) << 0, 1, 0, 0;
	// Start 10cm off the ground
	msckf.x.block<3,1>(4,0) << 0, 0, 0.1;
	//acc offset
	msckf.x.block<3,1>(4+3+3+3,0) << 0, 0, 0;



	VideoCapture cap;
	cap.open(0);

	// size
	cap.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

	namedWindow( "Features", 1 );

	Mat gray, prevGray;
	LKTracker tracker;
	double pX=0, pY=0;

	for(;;)
	{
		Mat frame;
		cap.grab();
		cap.retrieve(frame);

		cvtColor(frame, gray, COLOR_BGR2GRAY);
		if(prevGray.empty())
			gray.copyTo(prevGray);

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

		if ( points.cols() > 0 )
		{
			JacobiSVD<MatrixXd> svd( C, ComputeThinV );
			VectorXd V = svd.matrixV().rightCols<1>();

			VectorXd h = V.head<4>() / V(4);

			pX += h(2);
			pY += h(3);
			cout << endl;
			cout << endl;
			cout << "n Points: " << points.cols() << endl;
			cout << "Moved: " << h(2) << ", " << h(3) << endl;
			cout << "Total: " << pX << ", " << pY << endl;
		}

		for( int i = 0; i < tracker.points.size(); i++ )
		{
			line( frame, tracker.points[i], tracker.prevPoints[i], Scalar(0,255,0) );
			circle( frame, tracker.points[i], 2, Scalar(0,255,0) );
		}
		imshow("LK Demo", frame);

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
			break;
		}
		cv::swap(prevGray, gray);
	}
}