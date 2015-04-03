#ifndef _FEATURE_H_
#define _FEATURE_H_
#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "common.hpp"

using namespace cv;

class CameraDetector {
public:
	//
	// Interfaces for detection, extraction and matching
	//
	// Detector
	GoodFeaturesToTrackDetector detector;
	// Extractor
	Ptr<DescriptorExtractor> extractor;
	// Matcher
	BFMatcher matcher;

	// iterators pointing to CameraMeas of keypointOld
	std::vector<KeyPoint> keypointsOld;
	Mat descriptorsOld;

//public:
	// Constructor. TODO: add parameters
	CameraDetector( void );
	// detect features and add to feature vector
	void detectFeatures( const Mat& image, CameraMeasurements& cameraMeasurements );
};

#endif