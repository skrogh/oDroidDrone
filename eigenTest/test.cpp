#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

Matrix3d crossMat( const Vector3d& v ){
	Matrix3d m;
	m <<     0, -v(2),  v(1),
	      v(2),     0, -v(0),
	     -v(1),  v(0),     0;
	return m;
}

int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  Vector3d v(1.0, 2.0, 3.0);
  std::cout << v << " -> " << v.adjoint() << std::endl;
  std::cout << v << " -> " << crossMat(v) << std::endl;

  Quaternion<double> I1I_q;
  I1I_q = Quaternion<double>( 1, 0, 0, 0 );
  std::cout << Vector3d(I1I_q) << std::endl;
    I1I_q += Quaternion<double>( 1, 0, 0, 0 );
  std::cout << Vector3d(I1I_q) << std::endl;

}