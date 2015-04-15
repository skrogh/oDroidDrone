#ifndef _LKTRACKER_H_
#define _LKTRACKER_H_
#include "opencv2/opencv.hpp"
#include "common.hpp"
#include <ctype.h>
#include <random>


using namespace cv;

class LKTracker {
public:
	// Settings
	int MAX_COUNT;
	int N_OPTIMAL;
	int ROI_Y_SIZE;
	int ROI_X_SIZE;
	TermCriteria termcrit;
	std::uniform_int_distribution<int> roiDistX;
	std::uniform_int_distribution<int> roiDistY;
	std::uniform_int_distribution<int> deadDist;
	Size subPixWinSize;
	Size winSize;

	// Points and previous points
	vector<Point2f> points, prevPoints;

//public:
	// Constructor. TODO: add parameters
	LKTracker( void );
	// detect features
	void detectFeatures( const Mat& image, const Mat& prevImage );
	// Add features to feature vector
	void addFeatures( CameraMeasurements& cameraMeasurements );
};

#endif//_LKTRACKER_H_