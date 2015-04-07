#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "feature.hpp"
#include "common.hpp"

using namespace cv;

CameraDetector::CameraDetector( void ) {
	int maxCorners = 50;
	double qualityLevel = 0.02;
	double minDistance = 5;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	detector = GoodFeaturesToTrackDetector( maxCorners, qualityLevel,
			minDistance, blockSize, useHarrisDetector, k );

	extractor = DescriptorExtractor::create( "ORB" );

	matcher = BFMatcher(NORM_HAMMING, true);
}

void CameraDetector::detectFeatures( const Mat& image, CameraMeasurements& cameraMeasurements ) {
	keypointsOld = keypointsNew;
	detector.detect( image, keypointsNew );

	Mat descriptorsNew;
	extractor->compute( image, keypointsNew, descriptorsNew );

	matcher.match( descriptorsOld, descriptorsNew, matches );

	descriptorsOld = descriptorsNew;
}

void CameraDetector::addFeatures( CameraMeasurements& cameraMeasurements ) {
	cameraMeasurements.addFeatures( keypointsOld, keypointsNew,
		matches );
}