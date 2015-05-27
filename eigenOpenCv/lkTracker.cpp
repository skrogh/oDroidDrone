#include "lkTracker.hpp"
#include "opencv2/opencv.hpp"
#include "common.hpp"


using namespace cv;
using namespace std;

default_random_engine generator;


LKTracker::LKTracker( void )
{
	MAX_COUNT = 200;
	N_OPTIMAL = 50;
	ROI_Y_SIZE = 480/4;
	ROI_X_SIZE = 640/4;
	termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,10,0.03);
	roiDistX = uniform_int_distribution<int>(0,640-ROI_X_SIZE);
	roiDistY = uniform_int_distribution<int>(0,480-ROI_Y_SIZE);
	deadDist = uniform_int_distribution<int>(1,N_OPTIMAL/3);
	subPixWinSize =  Size(10,10);
	winSize = Size(31,31);
}

// detect features
void LKTracker::detectFeatures( const Mat& image, const Mat& prevImage )
{
	// Move current feature to the old ones
	std::swap(points, prevPoints);

	// Add new features, if we have too few. Add them from old image to old points
	if ( prevPoints.size() < N_OPTIMAL )
	{
		// get region of interest
		int roiX = roiDistX(generator);
		int roiY = roiDistY(generator);
		Point2f roiP( roiX, roiY );
		Mat roi(prevImage, Rect(roiX,roiY,ROI_X_SIZE,ROI_Y_SIZE));

		vector<Point2f> gfttPoints;
		//goodFeaturesToTrack(roi, gfttPoints, (N_OPTIMAL-prevPoints.size())/2, 0.01, 10, Mat(), 3, 0, 0.04);
		goodFeaturesToTrack(prevImage, gfttPoints, N_OPTIMAL, 0.01, 10, Mat(), 3, 0, 0.04);
		if ( !gfttPoints.empty() ) // Guard againts cornersubpixel throwing when no points
			cornerSubPix(prevImage, gfttPoints, subPixWinSize, Size(-1,-1), termcrit);
		for ( int i = 0; i < gfttPoints.size(); i++ ) {
			prevPoints.push_back(gfttPoints[i] + roiP);
		}
		prevPoints = gfttPoints;
	}

	// If
	if( !prevPoints.empty() )
	{
		vector<uchar> status;
		vector<float> err;
		calcOpticalFlowPyrLK(prevImage, image, prevPoints, points, status, err, winSize,
							 3, termcrit, 0, 0.001);
		size_t i, k;
		for( i = k = 0; i < points.size(); i++ )
		{
			// Skip this point, if it is invalid
			if( !status[i] || (deadDist(generator)==0) )
				continue;

			// Add it if it is valid (since i>k it is safe to use the poins vector for this)
			points[k] = points[i];
			prevPoints[k++] = prevPoints[i];
		}
		// k is the number of valid points
		points.resize(k);
		prevPoints.resize(k);
	}
}

// Add features to feature vector
void LKTracker::addFeatures( CameraMeasurements& cameraMeasurements )
{

}
