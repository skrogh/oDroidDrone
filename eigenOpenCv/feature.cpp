#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "feature.hpp"
#include "common.hpp"

using namespace cv;

CameraDetector::CameraDetector( void ) {
	int maxCorners = 500;
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

	std::vector<KeyPoint> keypointsNew;
	detector.detect( image, keypointsNew );

	Mat descriptorsNew;
	extractor->compute( image, keypointsNew, descriptorsNew );

	std::vector< DMatch > matches;
	matcher.match( descriptorsOld, descriptorsNew, matches );

	cameraMeasurements.addFeatures( keypointsOld, keypointsNew,
		matches );
}