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


void MSCKF::propagateState( double a_m[3], double g_m[3] ) {
	/*
	** Convert inputs
	*/
	Map<Vector3d> I_a_m( a_m, 3 );
	Map<Vector3d> I_g_m( g_m, 3 );

	/*
	** unpack state:
	*/
	// Rotation from global to inertial coordinates
	Quaternion<double> IG_q( x.segment<4>(0) );
	// Position (of inertial frame) in global coordinates
	Vector3d G_p( x.segment<3>(0+4) );
	// Velocity (of inertial frame) in global coordinates
	Vector3d G_v( x.segment<3>(0+4+3) );
	// Estimated gyro bias
	Vector3d b_g( x.segment<3>(0+4+3+3) );
	// Esitmated accelerometer bias
	Vector3d b_a( x.segment<3>(0+4+3+3+3) );

	/*
	** Calibrate
	*/
	// Accelerometer
	I_a = I_a_m - b_a;
	// Gyro
	I_g = I_g_m - b_g;

	/*
	** Propagate IMU
	*/
	// Rotation
	Quaternion<double> I1I_q;
	I1I_q = 
	// Translation

}
