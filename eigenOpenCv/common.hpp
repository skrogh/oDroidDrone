#ifndef _COMMON_H_
#define _COMMON_H_
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <Eigen/Dense>

typedef struct{
	Eigen::MatrixX2d z; // length is stored as number of elements in z
	bool isLost; // Set if it is no longer tracked in current frame
	// TODO: add feature description here?
	int linkLink;
} CameraMeas_t;

class CameraMeasurements {
public:
	std::list<CameraMeas_t> meas;
	//
	// Variables to hold feature info
	//
	std::vector< std::list<CameraMeas_t>::iterator > link;

	void addFeatures( const std::vector<cv::KeyPoint> &keypointsOld, const std::vector<cv::KeyPoint> &keypointsNew,
		const std::vector<cv::DMatch>& matches );
	void linkBack( std::list<CameraMeas_t>::iterator& feature, int linkLink );
	void addToFeature( std::list<CameraMeas_t>::iterator& feature, double x, double y );
	std::list<CameraMeas_t>::iterator addFeature( );
	std::list<CameraMeas_t>::iterator removeFeature( std::list<CameraMeas_t>::iterator& feature );
};

#endif //_COMMON_H_