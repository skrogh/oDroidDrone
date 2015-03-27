#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"


void printParams( cv::Algorithm* algorithm ) {
    std::vector<std::string> parameters;
    algorithm->getParams(parameters);

    for (int i = 0; i < (int) parameters.size(); i++) {
        std::string param = parameters[i];
        int type = algorithm->paramType(param);
        std::string helpText = algorithm->paramHelp(param);
        std::string typeText;

        switch (type) {
        case cv::Param::BOOLEAN:
            typeText = "bool";
            break;
        case cv::Param::INT:
            typeText = "int";
            break;
        case cv::Param::REAL:
            typeText = "real (double)";
            break;
        case cv::Param::STRING:
            typeText = "string";
            break;
        case cv::Param::MAT:
            typeText = "Mat";
            break;
        case cv::Param::ALGORITHM:
            typeText = "Algorithm";
            break;
        case cv::Param::MAT_VECTOR:
            typeText = "Mat vector";
            break;
        }
        std::cout << "Parameter '" << param << "' type=" << typeText << " help=" << helpText << std::endl;
    }
}

using namespace cv;


/** @function main */
int main( int argc, char** argv )
{
	if( argc != 3 )
	{ return -1; }

	Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

	if( !img_1.data || !img_2.data )
	{ return -1; }

	//-- Step 1: Detect the keypoints using GFTT Detector
	int maxCorners = 500;
	double qualityLevel = 0.02;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	Ptr<FeatureDetector> detector = FeatureDetector::create( "GFTT" );
	printParams( detector );
	//detector->set( "maxCorners", maxCorners );
	detector->set( "qualityLevel", qualityLevel );
	detector->set( "minDistance", minDistance );
	detector->set( "blockSize", blockSize );
	detector->set( "useHarrisDetector", useHarrisDetector );
	detector->set( "k", k );


	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector->detect( img_1, keypoints_1 );
	detector->detect( img_2, keypoints_2 );

	//-- Step 2: Calculate descriptors (feature vectors)
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create( "ORB" );
	printParams( extractor );

	Mat descriptors_1, descriptors_2;

	extractor->compute( img_1, keypoints_1, descriptors_1 );
	extractor->compute( img_2, keypoints_2, descriptors_2 );

	//-- Step 3: Matching descriptor vectors with a brute force matcher
	BFMatcher matcher(NORM_L2);
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );

	//-- Draw matches
	Mat img_matches;
	drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_matches );

	//-- Show detected matches
	imshow("Matches", img_matches );

	waitKey(0);

	return 0;
}