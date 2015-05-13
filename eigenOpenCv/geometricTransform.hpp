#ifndef _GEOMETRIC_TRANSFORM_H_
#define _GEOMETRIC_TRANSFORM_H_
#include <Eigen/Dense>

Eigen::Matrix<double,4,1> estimateSimilarTransform(
    const Eigen::Matrix2Xd& pointsFrom, const Eigen::Matrix2Xd& pointsTo );

#endif//_GEOMETRIC_TRANSFORM_H_
