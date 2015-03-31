#ifndef _FEATURE_H_
#define _FEATURE_H_

class FeatureDetector {
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
	std::vector< DMatch > matches;
public:
	// Constructor. TODO: add parameters
	FeatureDetector( void );
	// detect features and add to feature vector
	void detectFeatures( void );
};

#endif