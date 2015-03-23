#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "odometry.hpp"

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

Calib::Calib( ) {
	// 
	// Camera calibration
	//
	/* Projection */
	o_x = 0; o_y = 0;  // principal point [px,px]
	f_x = 1; f_y = 1;   // focal length [px,px]
	k1 = 0; k2 = 0;     // radial distortion parameters [n/u,n/u]
	t1 = 0; t2 = 0;     // tangential distortion parameters [n/u,n/u]
	/* Position */
	CI_q[4] = Quaternion<double>( 1, 0, 0, 0 ); // Rotation from intertial to camera coordinates. [unit quaternion]
	C_p_I = Vector3d( 0, 0, 0 );                // Position of inertial frame in camera coordinates [m,m,m]
	//
	// Physical properties
	//
	g = 0;           // Gravitational acceleration [m/s^2]
	delta_t = 1;     // Time between IMU data [s]
	imageOffset = 0; // Time delay for images [s]
	//
	// Noise levels
	//
	/* IMU TODO: add correct units */
	sigma_gc = 0; sigma_ac = 0;    // Standard deviation of IMU noise [rad/s, m/s^2]
	sigma_wgc = 0; sigma_wac = 0;  // Standard deviation of IMU random walk noise [rad/s, m/s^2]
	/* Camera */
	sigma_Im = 0;                  // Image noise
	/* distance sensor */
	sigma_dc = 0;                  // Standard deviation of distance sensor noise [m]
	//
	// Options
	//
	maxFrame = 0;                  // Maximum frames in FIFO
}

std::ostream& operator<<( std::ostream& out, const Calib& calib ) {
   return out << 
	"o_x: " << calib.o_x << " o_y: " << calib.o_y << "\n" <<
	"f_x: " << calib.f_x << " f_y: " << calib.f_y << "\n" <<
	"k1: " << calib.k1 << " k2: " << calib.k2 << "\n" <<
	"t1: " << calib.t1 << " t2: " << calib.t2 << "\n" <<
	"CI_q: " << (calib.CI_q).coeffs().transpose() << " C_p_I: " << calib.C_p_I.transpose() << "\n" <<
	"g: " << calib.g << "\n" <<
	"delta_t: " << calib.delta_t << "\n" <<
	"imageOffset: " << calib.imageOffset << "\n" <<
	"sigma_gc: " << calib.sigma_gc << " sigma_ac: " << calib.sigma_ac << "\n" <<
	"sigma_wgc: " << calib.sigma_wgc << " sigma_wac: " << calib.sigma_wac << "\n" <<
	"sigma_Im: " << calib.sigma_Im << "\n" <<
	"sigma_dc: " << calib.sigma_dc << "\n" <<
	"maxFrame: " << calib.maxFrame ;
}


MSCKF::MSCKF( Calib* cal ) {
	calib = cal;
	// init all states to 0;
	x = VectorXd(16).Zero(16);
	// init quaternion
	x.segment<4>(0) = Quaternion<double>( 1, 0, 0, 0 ).coeffs();
	// init state to known
	sigma = MatrixXd(15,15).Zero(15,15);

	// init delayed measurements
	I_a_dly = Vector3d( 0, 0, 0 );
	I_g_dly = Vector3d( 0, 0, 0 );
}

void MSCKF::propagateState( double a_m[3], double g_m[3] ) {
	/*
	** Convert inputs
	*/
	Map<Vector3d> I_a_m( a_m, 3 );
	Map<Vector3d> I_g_m( g_m, 3 );

	/*
	** Reusable constants:
	*/
	Vector3d G_g( 0, 0, -calib->g );

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
	Vector3d I_a = I_a_m - b_a;
	// Gyro
	Vector3d I_g = I_g_m - b_g;

	/*
	** Propagate IMU
	*/
	// Rotation
	Vector4d q0 = Vector4d( 0, 0, 0, 1 );
	Vector4d k1 = Omega( I_g_dly ) * q0 / 2.0;
	Vector4d k2 = Omega( ( I_g_dly + I_g ) / 2.0 ) * ( q0 + calib->delta_t/2.0 * k1 ) / 2.0;
	Vector4d k3 = Omega( ( I_g_dly + I_g ) / 2.0 ) * ( q0 + calib->delta_t/2.0 * k2 ) / 2.0;
	Vector4d k4 = Omega( I_g ) * ( q0 + calib->delta_t * k3 ) / 2.0;

	Quaternion<double> I1I_q(
			Vector4d( 0, 0, 0, 1 )
			+ calib->delta_t/6.0 * ( k1 + 2*k2, + 2*k3 + k4 )
	);
	I1I_q.normalize();

	Quaternion<double> I1G_q = I1I_q * IG_q;

	// Translation
	Vector3d G_a = I1G_q._transformVector( I_a ) + G_g;

	Vector3d s = calib->delta_t/2.0 * (
			I1G_q.conjugate()._transformVector( I_a ) + I_a_dly
	);
	Vector3d y = calib->delta_t/2.0 * s;

	Vector3d G_v1 = G_v + IG_q.conjugate()._transformVector( s ) + G_g * calib->delta_t;

	Vector3d G_p1 = G_p + G_v * calib->delta_t
			+ IG_q.conjugate()._transformVector( y )
			+ G_g * calib->delta_t * calib->delta_t / 2.0;

	/*
	** Repack state
	*/
	// Rotation from global to inertial coordinates
	x.segment<4>(0) = IG_q.coeffs();
	// Position (of inertial frame) in global coordinates
	x.segment<3>(0+4) = G_p1;
	// Velocity (of inertial frame) in global coordinates
	x.segment<3>(0+4+3) = G_v1;
	// No change to biases
}
