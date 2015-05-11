#include "geometricTransform.hpp"

using namespace Eigen;

Matrix<double,4,1> estimateRigidTransform(
    const Matrix2Xd& pointsFrom, const Matrix2Xd& pointsTo ) {
  Matrix<double, Dynamic, 5> C( pointsTo.cols()*2, 5 );
  for ( int i = 0; i < pointsTo.cols(); i++ )
  {
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
  return h;
}
