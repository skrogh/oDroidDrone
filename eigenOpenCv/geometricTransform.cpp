#include "geometricTransform.hpp"
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>


using namespace Eigen;

Matrix3d computeSimilarity(
    const Matrix2Xd& pointsFrom, const Matrix2Xd& pointsTo ) {
  Matrix<double, Dynamic, 5> C( pointsTo.cols()*2, 5 );
  for ( int i = 0; i < pointsTo.cols(); i++ ) {
    const double &x = pointsTo(0,i);
    const double &y = pointsTo(1,i);
    const double &x_ = pointsFrom(0,i);
    const double &y_ = pointsFrom(1,i);
    C.row( i*2 )    << -y,  x,  0, -1,  y_;
    C.row( i*2 + 1) <<  x,  y,  1,  0, -x_;
  }
  // solve for transformation
  JacobiSVD<MatrixXd> svd( C, ComputeThinV );
  VectorXd V = svd.matrixV().rightCols<1>();
  // find transformation
  Matrix<double,4,1> h = V.head<4>() / V(4);

  Matrix3d T;
  T << h(0), -h(1), 0,
       h(1),  h(0), 0,
       h(2),  h(3), 1;
  return T;
}

Matrix2Xd normalizePoints( const Matrix2Xd& points, const VectorXd& indices,
    Ref<Matrix3d> T ) {
  Matrix2Xd pointsOut( 2, indices.rows() );

  // Calculate center
  Vector2d center(0,0);
  for ( int j = 0; j < indices.rows(); j++ ) {
    int i = indices(j);
    center += points.col(i);
    pointsOut.col(j) = points.col(i);
  }
  center /= indices.rows();

  // Calculate mean distance from center
  double meanDistanceFromCenter = 0;
  for ( int i = 0; i < pointsOut.cols(); i++ ) {
    pointsOut.col(i) -= center;
    meanDistanceFromCenter += pointsOut.col(i).norm();
  }
  meanDistanceFromCenter /= pointsOut.cols();

  // Calculate scale
  double scale;
  if ( meanDistanceFromCenter > 0 ) {
    scale = sqrt(2) / meanDistanceFromCenter;
  } else {
    scale = 1;
  }

  // Compute matrix to translate and scale points
  T << scale, 0, -scale*center(0),
       0, scale, -scale*center(1),
       0,     0,                1;

  // scale points
  if ( points.cols() > 2 ) {
    pointsOut = T.block<2,2>(0,0) * points;
    for( int i = 0; i < pointsOut.cols(); i++ ) {
      pointsOut.col(i) += T.block<2,1>(0,2);
    }
  } else {
    pointsOut *= scale;
  }

  return pointsOut;
}

Matrix3d computeTform(
    const Matrix2Xd& pointsFrom, const Matrix2Xd& pointsTo,
    const VectorXd& indices ) {
  Matrix3d normMatFrom;
  Matrix3d normMatTo;
  Matrix3d T = computeSimilarity(
    normalizePoints( pointsFrom, indices, normMatFrom ),
    normalizePoints( pointsTo, indices, normMatTo ) );
  return normMatFrom.transpose() * ( T * normMatTo.transpose().inverse() );
}

VectorXd evaluateTform( const Matrix2Xd& pointsFrom, const Matrix2Xd& pointsTo,
    const Matrix3d& T, double threshold ) {
  Matrix2Xd pt( pointsFrom.rows(), pointsFrom.cols() );
  pt = T.block<2,2>(0,0).transpose() * pointsFrom;
  for( int i = 0; i < pointsFrom.cols(); i++ ) {
    pt.col(i) += T.block<2,1>(0,2);
  }

  pt = pt - pointsTo;
  VectorXd dis( pt.cols() );
  for ( int i = 0; i < dis.rows(); i++ ) {
    dis(i) = pt.col(i).norm();
    if( dis(i) > threshold )
      dis(i) = threshold;
  }

  return dis;
}

std::default_random_engine msacGenerator;
Matrix3d msac( const Eigen::Matrix2Xd& pointsFrom, const Eigen::Matrix2Xd& pointsTo,
    int maxNumTrials, double confidence, double maxDistance ) {
  double threshold = maxDistance;
  int numPts = pointsFrom.cols();
  int idxTrial = 1;
  int numTrials = maxNumTrials;
  double maxDis = threshold * numPts;
  double bestDist = maxDis;
  Matrix3d bestT;
  bestT << 1, 0, 0,   0, 1, 0,  0, 0, 1;

  int index1;
  int index2;
  // Get two random, different numbers in [0:pointsFrom.cols()-1]
  std::uniform_int_distribution<int> distribution1( 0, pointsFrom.cols()-1 );
  std::uniform_int_distribution<int> distribution2( 0, pointsFrom.cols()-2 );
  while ( idxTrial <= numTrials ) {
    // Get two random, different numbers in [0:pointsFrom.cols()-1]
    index1 = distribution1( msacGenerator );
    index2 = distribution2( msacGenerator );
    if ( index2 >= index1 )
      index2++;
    Vector2d indices( index1, index2 );

    /*std::cout << "indices: " << indices.transpose()
    << " pointsFrom.cols: " << pointsFrom.cols()
    << " pointsTo.cols: " << pointsTo.cols() << std::endl;*/

    // Get T form Calculated from this set of points
    Matrix3d T = computeTform( pointsFrom, pointsTo, indices );

    VectorXd dis = evaluateTform( pointsFrom, pointsTo, T, threshold );

    double accDis = dis.sum();

    if ( accDis < bestDist ) {
      bestDist = accDis;
      bestT = T;
    }
    idxTrial++;
  }

  threshold *= dis.mean();
  VectorXd dis = evaluateTform( pointsFrom, pointsTo, bestT, threshold );
  int numInliers = 0;
  for ( int i = 0; i < dis.rows(); i++ ){
    if ( dis(i) < threshold )
      numInliers++;
  }
  VectorXd inliers( numInliers );
  int j = 0;
  for ( int i = 0; i < dis.rows(); i++ ){
    if ( dis(i) < threshold )
      inliers(j++) = i;
  }

  Matrix3d T;
  if ( numInliers >= 2 )
    T = computeTform( pointsFrom, pointsTo, inliers );
  else
    T << 1, 0, 0,  0, 1, 0,  0, 0, 1;

  return T;
}

Eigen::Matrix<double,4,1> estimateSimilarTransform(
    const Eigen::Matrix2Xd& pointsFrom, const Eigen::Matrix2Xd& pointsTo ) {
  Matrix<double,4,1> h(4,1);
  VectorXd indices( pointsFrom.cols() );
  for ( int i = 0; i < indices.rows(); i++ ) {
    indices(i) = i;
  }
  //Matrix3d T = computeSimilarity( pointsFrom, pointsTo ); // No ransac'n
  Matrix3d T = msac( pointsTo, pointsFrom, 20, 99, 1 );

  h(0) = T(0,0);
  h(1) = T(1,0);
  h(2) = T(2,0);
  h(3) = T(2,1);

  return h;
}
