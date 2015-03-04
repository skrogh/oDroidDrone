#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

int main()
{
	// Open camera
	VideoCapture cap(0);
	// check success
	if ( !cap.isOpened() )
		return -1;

	// set camera parameters:
	// size
	cap.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, 240 );
	// framerate
	system( "v4l2-ctl --set-parm=125" );
	// exposure, gain, etc.


	Mat edges;
	Mat frame;

/*
	Ptr<FeatureDetector> detector(
		new GridAdaptedFeatureDetector(
		new DynamicAdaptedFeatureDetector(
		new FastAdjuster( 20, true ), 1, 3, 10 ),
		200, 8, 6 )
	);
*/
	Ptr<FeatureDetector> detector(
		new GridAdaptedFeatureDetector(
		new FastFeatureDetector( 20 ), 500, 8, 6
	) );


	std::vector<KeyPoint> keypoints;

	vector<int> compressionParams;
	compressionParams.push_back( CV_IMWRITE_PNG_COMPRESSION );
	compressionParams.push_back( 9 );
	
	while( 1 ) {
		// get time
		struct timeval tv;
		struct timezone tz = {
			.tz_minuteswest = 0,
			.tz_dsttime = 0
		};

		// get new frame
		cap >> frame;
		// get time
		gettimeofday( &tv, &tz );
		char nameString[50];
		sprintf( nameString, "opencv_test_images/img-s%ld.%06ld.png", tv.tv_sec, tv.tv_usec );

		// convert to mono
		cvtColor( frame, edges, CV_RGB2GRAY );
		// save image
		imwrite( nameString, edges, compressionParams );

//		detector->detect( edges, keypoints );
//		drawKeypoints( edges, keypoints, edges, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

		// show image
		imshow( "edges", edges );
		// wait a while (gives CV time to show image, and desk to save image)
		if( waitKey( 80 ) >= 0 ) break;
	}
	return 0;
}
