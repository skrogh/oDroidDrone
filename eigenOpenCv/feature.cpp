#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <Eigen/Dense>
#include "feature.hpp"
#include "odometry.hpp"

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

void CameraFeatures::detectFeatures( const Mat& image, std::list<CameraMeas_t>& meas ) {

	std::vector<KeyPoint> keypointsNew;
	detector.detect( image, keypointsNew );

	Mat descriptorsNew;
	extractor->compute( image, keypointsNew, descriptorsNew );

	std::vector< DMatch > matches;
	matcher.match( descriptorsOld, descriptorsNew, matches );


	// Go through all measurements and mark all of them as lost
	for ( std::list<CameraMeas_t>::iterator meas_j = meas.begin(); meas_j != meas.end(); ++meas_j ) {
		meas_j->isLost = true;
	}

	// mark all new links as no link
	std::vector< std::list<CameraMeas_t>::iterator > linkNew;
	linkNew.assign( keypointsNew.size(), meas.end() );

	// Go through all features with a match, add them at the correct place
	for ( int i = 0; i < matches.size(); i++ ) {
		// Check if this feature is in meas
		if ( linkOld[matches[i].queryIdx] != meas.end() ) {
			//
			// Feature is already in meas
			//

			// Link it
			linkNew[matches[i].trainIdx] = linkOld[matches[i].queryIdx];
			// Add feature TODO: consider if theis matrix should be made a vector
			MatrixX2d& z = linkNew[matches[i].trainIdx]->z;
			z.conservativeResize ( z.rows() + 1, NoChange );
			z.block<1,2>( z.rows()-1, 0 ) <<
					keypointsNew[matches[i].trainIdx].pt.x,
					keypointsNew[matches[i].trainIdx].pt.y;
			// It is no longer lost
			linkNew[matches[i].trainIdx]->isLost = false;
		} else {
			//
			// Feature is new. In this case both old and new is to be added
			//

			// Add a new entry
			CameraMeas_t z;
			std::list<CameraMeas_t>::iterator newFeature = meas.insert( meas.begin(), z );
			newFeatur->z = Eigen::MatrixX2d( 2, 2 );
			newFeatur->z << 
					keypointsOld[matches[i].queryIdx].pt.x, keypointsOld[matches[i].queryIdx].pt.y,
					keypointsNew[matches[i].trainIdx].pt.x, keypointsNew[matches[i].trainIdx].pt.y;
			// not lost (we just got it duh)
			newFeature->isLost = false;
			// Link it
			linkNew[matches[i].trainIdx] = newFeature;
		}
	}

	keypointsOld = keypointsNew;
	descriptorsOld = descriptorsNew;
	linkOld = linkNew;
}