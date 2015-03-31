#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "feature.hpp"

using namespace cv;

CameraFeatures::CameraFeatures( void ) {
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

void CameraFeatures::detectFeatures( void ) {
	Mat image;

	std::vector<KeyPoint> keypointsNew;
	detector.detect( image, keypointsNew );

	Mat descriptorsNew;
	extractor->compute( imgage, keypointsNew, descriptorsNew );

	matcher.match( descriptorsOld, descriptorsNew, matches );

	
}