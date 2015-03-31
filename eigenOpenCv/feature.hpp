#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <Eigen/Dense>
#include "feature.hpp"
#ifndef _FEATURE_H_
#define _FEATURE_H_

using namespace cv;

class CameraFeatures {
private:
	//
	// Interfaces for detection, extraction and matching
	//
	// Detector
	GoodFeaturesToTrackDetector detector;
	// Extractor
	Ptr<DescriptorExtractor> extractor;
	// Matcher
	BFMatcher matcher;

	//
	// Variables to hold results
	//
	std::vector<KeyPoint> keypointsOld;
	Mat descriptorsOld;
	// iterators pointing to CameraMeas of keypointOld
	std::vector< std::list<CameraMeas_t>::iterator > linksOld;

public:
	// Constructor. TODO: add parameters
	CameraFeatures( void );
	// detect features and add to feature vector
	void detectFeatures( const Mat& image, std::list<CameraMeas_t>& meas );
};

#endif