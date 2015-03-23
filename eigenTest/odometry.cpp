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

//
// Omega( v ) = -[v] v
//               -v  0
//
Matrix4d Omega( const Vector3d& v ){
	Matrix4d m;
	m <<     0,  v(2), -v(1),  v(0),
	     -v(2),     0,  v(0),  v(1),
	      v(1), -v(0),     0,  v(2),
	     -v(0), -v(1), -v(2),     0;
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
	Vector4d q = Vector4d( 0, 0, 0, 1 );
	Vector4d k1 = Omega( I_g_dly ) * q0 / 2.0;
	Vector4d k2 = Omega( ( I_g_dly + I_g ) / 2.0 ) * ( q0 + calib.delta_t/2.0 * k1 ) / 2.0;
	Vector4d k3 = Omega( ( I_g_dly + I_g ) / 2.0 ) * ( q0 + calib.delta_t/2.0 * k2 ) / 2.0;
	Vector4d k4 = Omega( I_g ) * ( q0 + calib.delta_t * k3 ) / 2.0;

	Quaternion<double> I1I_q(
			Vector4d( 0, 0, 0, 1 )
			+ calib.delta_t/6.0 * ( k1 + 2*k2, + 2*k3 + k4 )
	);
	I1I_q.normalize();

	Quaternion<double> I1G_q = I1I_q * IG_q;

	// Translation
	Vector3d G_a = 

}
