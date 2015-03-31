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
	/*while( ( key = cv::waitKey(1) ) != 27 ) {*/
	while( key++ < 20/4*60 ) {
		/*
		cap.grab();
		cap.grab();
		cap.grab();
		cap.grab();
		cap.grab();
		cap.retrieve( image );
		*/

		cap.read( image );


		cameraDetector.detectFeatures( image, cameraMeasurements );

		for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ) {
			if ( ( meas_j->z.rows() > 5 ) || ( meas_j->isLost ) ) {
				meas_j = cameraMeasurements.removeFeature( meas_j );
			} else {
				++meas_j;
			}
		}

/*
		if( key == '\n' ) {
			for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ) {
				meas_j = cameraMeasurements.removeFeature( meas_j );
			}
		}
*/

		std::cout << cameraMeasurements.meas.size() << " Total features" << std::endl;
		// Iterate over meas and draw all non lost elements:
		/*
		for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ++meas_j ) {
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
		}*/

		//cv::imshow("Matches", image );
	}


	return 0;
}
