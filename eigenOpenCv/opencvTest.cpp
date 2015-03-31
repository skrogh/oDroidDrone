#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "feature.hpp"
#include "odometry.hpp"

/** @function main */
int main( int argc, char** argv )
{
	cv::VideoCapture cap(0);
	cv::Mat image;
	std::list<CameraMeas_t> meas;

	CameraFeatures cameraFeatures = CameraFeatures( );


	cv::namedWindow( "Matches" );

	while( cv::waitKey(0) != 27 ) {
		cap.grab();
		cap.grab();
		cap.grab();
		cap.grab();
		cap.grab();
		cap.retrieve( image );

		cameraFeatures.detectFeatures( image, meas );

		// Iterate over meas and draw all non lost elements:
		for ( std::list<CameraMeas_t>::iterator meas_j = meas.begin(); meas_j != meas.end(); ++meas_j ) {
			if ( !meas_j->isLost ) {
				Eigen::MatrixX2d& z = meas_j->z;
				cv::Point pt = Point( z( z.rows()-1, 0 ), z( z.rows()-1, 1 ) );
				cv::circle( image, pt, 4, Scalar( 255, 0, 0 ) );
				for( int i = 0; i < z.rows() - 1; i++ ) {
					cv::Point pt1 = Point( z( i, 0 ), z( i, 1 ) );
					cv::Point pt2 = Point( z( i+1, 0 ), z( i+1, 1 ) );
					cv::line( image, pt1, pt2, Scalar( 255, 0, 0 ) );
				}

			}
		}

		cv::imshow("Matches", image );
	}


	return 0;
}
