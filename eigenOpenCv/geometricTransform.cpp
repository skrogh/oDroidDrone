#include "geometricTransform.hpp"

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
    int i =indices(j);
    center += points.col(i);
    pointsOut.col(i) = points.col(i);
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
    pointsOut = T * points;
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
    normalizePoints( pointsFrom, indices, normMatTo ) );
  return normMatFrom.transpose() * ( T * normMatTo.transpose().inverse() );
}


Eigen::Matrix<double,4,1> estimateSimilarTransform(
    const Eigen::Matrix2Xd& pointsFrom, const Eigen::Matrix2Xd& pointsTo ) {
  Matrix<double,4,1> h(4,1);
  VectorXd indices( pointsFrom.cols() );
  for ( int i = 0; i < indices.rows(); i++ ) {
    indices(i) = i;
  }
  Matrix3d T = computeTform( pointsFrom, pointsTo, indices );
  h(0) = T(0,0);
  h(1) = T(1,0);
  h(2) = T(2,0);
  h(3) = T(2,1);
}
