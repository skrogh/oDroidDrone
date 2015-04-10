#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "feature.hpp"
#include "odometry.hpp"

/** @function main */
int main( int argc, char** argv )
{
	cv::VideoCapture cap(0);
	cv::Mat image;

	CameraMeasurements cameraMeasurements;
	CameraDetector cameraDetector = CameraDetector( );


	cv::namedWindow( "Matches" );

	int key = 0;
	while( ( key = cv::waitKey(1) ) != 27 ) {
		cap.grab();
/*		cap.grab();
		cap.grab();
		cap.grab();
		cap.grab();*/
		cap.retrieve( image );
		cvtColor(image, image, CV_BGR2GRAY);

		cameraDetector.detectFeatures( image, cameraMeasurements );
		cameraDetector.addFeatures( cameraMeasurements );

		if( key == '\n' ) {
			for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ) {
				meas_j = cameraMeasurements.removeFeature( meas_j );
			}
		}

		// Iterate over meas and draw all non lost elements:
		for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ++meas_j ) {
			if ( !meas_j->isLost && (meas_j->z.rows()>3) ) {
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
