#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <cmath>

#include "odometry.hpp"
#include "common.hpp"

using namespace Eigen;


//
// Chi square lookup for 95% confidence. TODO: calculate at runtime so size fits with max frame fifo length
//
const double chi2Inv[] = {
	0,
	3.841458821,
	5.991464547,
	7.814727903,
	9.487729037,
	11.07049769,
	12.59158724,
	14.06714045,
	15.50731306,
	16.9189776,
	18.30703805,
	19.67513757,
	21.02606982,
	22.36203249,
	23.6847913,
	24.99579014,
	26.2962276,
	27.58711164,
	28.86929943,
	30.14352721,
	31.41043284,
	32.67057334,
	33.92443847,
	35.17246163,
	36.4150285,
	37.65248413,
	38.88513866,
	40.11327207,
	41.33713815,
	42.5569678,
	43.77297183
};


//
// calculates a to the power of i. i is a positive, non zero, integer
//
inline double iPow( double a, unsigned int i ) {
	for( i=i; i>1; i-- )
		a *= a;
	return a;
}


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


//
//
//
Vector2d cameraProject( double X, double Y, double Z, Calib* calib ) {
	double& o_x = calib->o_x;
	double& o_y = calib->o_y;
	double& f_x = calib->f_x;
	double& f_y = calib->f_y;
	double& k1 = calib->k1;
	double& k2 = calib->k2;
	double& t1 = calib->t1;
	double& t2 = calib->t2;

	double u = X/Z;
	double v = Y/Z;
	double r = u*u + v*v;
	double dr = 1 + k1*r + k2*r*r;
	Vector2d dt( 2*u*v*t1 + ( r + 2*u*u ) * t2, 2*u*v*t2 + ( r + 2*v*v ) * t1 );

	return Vector2d( o_x + f_x * ( dr * u + dt(0) ), o_y + f_y * ( dr * v + dt(1) ) );
}

//
// Jacobian of h (camera model)
//
Matrix<double,2,3> jacobianH( double X, double Y, double Z, Calib* calib ) {
	double& o_x = calib->o_x;
	double& o_y = calib->o_y;
	double& f_x = calib->f_x;
	double& f_y = calib->f_y;
	double& k1 = calib->k1;
	double& k2 = calib->k2;
	double& t1 = calib->t1;
	double& t2 = calib->t2;


	Matrix<double,2,3> m;
	m <<
	f_x*( 
		( k2*iPow( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2), 2 ) + k1*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) + 1)/Z
		+ ( X*( ( 2*X*k1 )/iPow(Z,2) + ( 4*X*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) )/iPow(Z,2) ) )/Z
		+ ( 6*X*t2 )/iPow(Z,2)
		+ ( 2*Y*t1 )/iPow(Z,2)
	),
	f_x*( 
		( X*( ( 2*Y*k1 )/iPow(Z,2) + ( 4*Y*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) )/iPow(Z,2) ) )/Z
		+ ( 2*X*t1 )/iPow(Z,2)
		+ ( 2*Y*t2 )/iPow(Z,2)
	),
	-f_x*(
		t2*( ( 6*iPow(X,2) )/iPow(Z,3) + ( 2*iPow(Y,2) )/iPow(Z,3) )
		+ ( X*( k1*( ( 2*iPow(X,2) )/iPow(Z,3) + ( 2*iPow(Y,2) )/iPow(Z,3) ) + 2*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) )*( ( 2*iPow(X,2) )/iPow(Z,3) + ( 2*iPow(Y,2) )/iPow(Z,3) ) ) )/Z
		+ ( X*( k2*iPow( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2), 2) + k1*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) + 1 ) )/iPow(Z,2)
		+ ( 4*X*Y*t1 )/iPow(Z,3)
	),

	f_y*(
		( Y*( ( 2*X*k1 )/iPow(Z,2) + ( 4*X*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) )/iPow(Z,2) ) )/Z
		+ ( 2*X*t1 )/iPow(Z,2)
		+ ( 2*Y*t2 )/iPow(Z,2)
	),

	f_y*(
		( k2*iPow( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2), 2 ) + k1*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) + 1 )/Z
		+ ( Y*( ( 2*Y*k1 )/iPow(Z,2) + ( 4*Y*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) )/iPow(Z,2) ) )/Z
		+ ( 2*X*t2 )/iPow(Z,2)
		+ ( 6*Y*t1 )/iPow(Z,2)
	),

	-f_y*(
		t1*( ( 2*iPow(X,2) )/iPow(Z,3) + ( 6*iPow(Y,2) )/iPow(Z,3) )
		+ ( Y*( k1*( ( 2*iPow(X,2) )/iPow(Z,3) + ( 2*iPow(Y,2) )/iPow(Z,3) ) + 2*k2*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) )*( ( 2*iPow(X,2) )/iPow(Z,3) + ( 2*iPow(Y,2) )/iPow(Z,3) ) ) )/Z
		+ ( Y*( k2*iPow( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2), 2 ) + k1*( iPow(X,2)/iPow(Z,2) + iPow(Y,2)/iPow(Z,2) ) + 1 ) )/iPow(Z,2)
		+ ( 4*X*Y*t2 )/iPow(Z,3)
	);
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
	CI_q = Quaternion<double>( 1, 0, 0, 0 ); // Rotation from intertial to camera coordinates. [unit quaternion]
	C_p_I = Vector3d( 0, 0, 0 );                // Position of inertial frame in camera coordinates [m,m,m]
	//
	// Physical properties
	//
	g = 0;           // Gravitational acceleration [m/s^2]
	delta_t = 1;     // Time between IMU data [s]
	imageOffset.tv_sec = 0; // Time delay for images [s]
	imageOffset.tv_usec = 0; // Time delay for images [us]
	//
	// Noise levels
	//
	/* IMU TODO: add correct units */
	sigma_gc = 0; sigma_ac = 0;    // Standard deviation of IMU noise [rad/s, m/s^2]
	sigma_wgc = 0; sigma_wac = 0;  // Standard deviation of IMU random walk noise [rad/s, m/s^2]
	/* Camera */
	sigma_Im = 0;                  // Image noise
	/* height sensor */
	sigma_hc = 0;                  // Standard deviation of height sensor noise [m]
	//
	// Options
	//
	maxFrame = ODO_MAX_FRAMES;                  // Maximum frames in FIFO
	minFrame = 0;
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
	"imageOffset: " << calib.imageOffset.tv_usec << "us\n" <<
	"sigma_gc: " << calib.sigma_gc << " sigma_ac: " << calib.sigma_ac << "\n" <<
	"sigma_wgc: " << calib.sigma_wgc << " sigma_wac: " << calib.sigma_wac << "\n" <<
	"sigma_Im: " << calib.sigma_Im << "\n" <<
	"sigma_hc: " << calib.sigma_hc << "\n" <<
	"maxFrame: " << calib.maxFrame << "\n" <<
	"minFrame: " << calib.minFrame ;
}


MSCKF::MSCKF( Calib* cal ) {
	calib = cal;
	// init all states to 0;
	x = VectorXd::Zero(16);
	// init quaternion
	x.segment<4>(0) = Quaternion<double>( 1, 0, 0, 0 ).coeffs();
	// init state to known
	sigma = MatrixXd::Zero(15,15);

	// init delayed measurements
	I_a_dly = Vector3d( 0, 0, 0 );
	I_g_dly = Vector3d( 0, 0, 0 );
}

std::ostream& operator<<( std::ostream& out, const MSCKF& msckf ) {
	return out <<
	"IG_q: " << msckf.x.segment<4>(0).transpose() << "\n" <<
	"G_p: " << msckf.x.segment<3>(0+4).transpose() << "\n" <<
	"G_v: " << msckf.x.segment<3>(0+4+3).transpose() << "\n" <<
	"b_g: " << msckf.x.segment<3>(0+4+3+3).transpose() << "\n" <<
	"b_a: " << msckf.x.segment<3>(0+4+3+3+3).transpose();
}

void MSCKF::propagate( double a_m[3], double g_m[3] ) {
	/*
	** Convert inputs
	*/
	Vector3d I_a_m( a_m[0], a_m[1], a_m[2] );
	Vector3d I_g_m( g_m[0], g_m[1], g_m[2] );

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
	Vector3d G_a = I1G_q.conjugate()._transformVector( I_a ) + G_g;

	Vector3d s = calib->delta_t/2.0 * (
			I1I_q.conjugate()._transformVector( I_a ) + I_a_dly
	);
/*	std::cout << "debug1: " << I1G_q.conjugate()._transformVector( I_a ) << std::endl;
	std::cout << "debug2: " << I_a_dly << std::endl;
	std::cout << "debug3: " << I1G_q.conjugate()._transformVector( I_a )  + I_a_dly << std::endl;
	std::cout << "debug4: " << calib->delta_t/2.0 * ( I1G_q.conjugate()._transformVector( I_a )  + I_a_dly ) << std::endl;
	std::cout << "debug5: " << calib->delta_t * ( I1G_q.conjugate()._transformVector( I_a )  + I_a_dly ) << std::endl;
	std::cout << "debug6: " << calib->delta_t * ( I1G_q.conjugate()._transformVector( I_a )  + I_a_dly ) /2.0 << std::endl;
	std::cout << "debug7: " << s << std::endl;*/

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

	/*
	** Propagate error
	*/

	// aliases
	Matrix3d R = IG_q.toRotationMatrix();
	Matrix3d R1 = I1G_q.toRotationMatrix();
	Matrix3d R_ = R.transpose();
	Matrix3d R1_ = R1.transpose();

	Matrix3d Phi_qq = Matrix3d::Identity();
	Matrix3d Phi_pq = -crossMat( R_ * y );
	Matrix3d Phi_vq = -crossMat( R_ * s );

	Matrix3d Phi_qbg = -calib->delta_t * ( R1_ + R_ ) / 2.0;
	Matrix3d Phi_vbg = calib->delta_t * calib->delta_t * crossMat( G_a - G_g ) * ( R1_ + R_ ) / 4.0;
	Matrix3d Phi_pbg = calib->delta_t * Phi_vbg / 2.0;

	Matrix3d Phi_qba = Matrix3d::Zero(); // assuming no gravitational effect on gyro
	Matrix3d Phi_vba = -calib->delta_t * ( R1_ + R_ ) / 2.0;
	Matrix3d Phi_pba = calib->delta_t * Phi_vba / 2.0;

	Matrix<double,15,15> Phi_I;
	Phi_I <<
			          Phi_qq,     Matrix3d::Zero(),                      Matrix3d::Zero(),              Phi_qbg,              Phi_qba,
			          Phi_pq, Matrix3d::Identity(), calib->delta_t * Matrix3d::Identity(),              Phi_pbg,              Phi_pba,
			          Phi_vq,     Matrix3d::Zero(),                  Matrix3d::Identity(),              Phi_vbg,              Phi_vba,
			Matrix3d::Zero(),     Matrix3d::Zero(),                      Matrix3d::Zero(), Matrix3d::Identity(),     Matrix3d::Zero(),
			Matrix3d::Zero(),     Matrix3d::Zero(),                      Matrix3d::Zero(),     Matrix3d::Zero(), Matrix3d::Identity();

	Matrix<double,15,15> N_c;
	Matrix<double,15,15> Q_d;
	Matrix<double,15,1> N_c_diag;
	N_c_diag <<
			calib->sigma_gc*calib->sigma_gc*Vector3d(1,1,1),
			Vector3d(0,0,0),
			calib->sigma_ac*calib->sigma_ac*Vector3d(1,1,1),
			calib->sigma_wgc*calib->sigma_wgc*Vector3d(1,1,1),
			calib->sigma_wac*calib->sigma_wac*Vector3d(1,1,1);
	N_c = N_c_diag.asDiagonal();
	Q_d = calib->delta_t * Phi_I * N_c * Phi_I.transpose() + N_c;

	// do the update
	sigma.block<15,15>(0,0) = Phi_I * sigma.block<15,15>(0,0) * Phi_I.transpose() + Q_d;
	sigma.block(0,15,15,sigma.cols()-15) = Phi_I * sigma.block(0,15,15,sigma.cols()-15);
	sigma.block(15,0,sigma.rows()-15,15) = sigma.block(0,15,15,sigma.cols()-15).transpose();

	/*
	** update delayed variables
	*/
	I_a_dly = I_a;
	I_g_dly = I_g;
}

void MSCKF::augmentState( void ) {
	/*
	** Resize state and sigma, so new data can fit
	*/

	// State
	x.conservativeResize( x.rows() + ODO_STATE_FRAME_SIZE, NoChange );
	x.block<ODO_STATE_FRAME_SIZE,1>( x.rows() - ODO_STATE_FRAME_SIZE, 0 ) =
			x.block<ODO_STATE_FRAME_SIZE,1>( 0, 0 ); 

	// Covariance
	sigma.conservativeResize( sigma.rows() + ODO_SIGMA_FRAME_SIZE,
			sigma.cols() + ODO_SIGMA_FRAME_SIZE
	);
	/*
	** Fast way of doing:
	** J = [ I_9, 0_(9,6), 0_(9,n*9) ]
	** where the frame size in sigma is 9 and n is the previous number of frames
	** Then do:
	** Sigma = [ Sigma, Sigma*J', J*Sigma, J*Sigma*J']
	** Since J is the identitymatrix and a lot of zeros, this is straight-up copying:
	*/
	sigma.block<ODO_SIGMA_FRAME_SIZE,ODO_SIGMA_FRAME_SIZE>( sigma.rows() - ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE ) =
			sigma.block<ODO_SIGMA_FRAME_SIZE,ODO_SIGMA_FRAME_SIZE>( 0, 0 );
	sigma.block( 0, sigma.cols() - ODO_SIGMA_FRAME_SIZE, sigma.rows() - ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE ) =
			sigma.block( 0, 0, sigma.rows() - ODO_SIGMA_FRAME_SIZE, ODO_SIGMA_FRAME_SIZE );
	sigma.block( sigma.rows() - ODO_SIGMA_FRAME_SIZE, 0, ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE) =
			sigma.block( 0, 0, ODO_SIGMA_FRAME_SIZE, sigma.cols() - ODO_SIGMA_FRAME_SIZE);
}

void MSCKF::removeOldStates( int n ) {
	//
	// Skip if n < 1
	//
	if ( n < 1 )
		return;


	/*
	** Remove the n oldest frames from the state and covariance
	*/
	x.block( ODO_STATE_SIZE, 0, x.rows() - ODO_STATE_SIZE - n * ODO_STATE_FRAME_SIZE, 1 ) =
			x.block( ODO_STATE_SIZE + n * ODO_STATE_FRAME_SIZE, 0, x.rows() - ODO_STATE_SIZE - n * ODO_STATE_FRAME_SIZE, 1 );
	x.conservativeResize( x.rows() - n * ODO_STATE_FRAME_SIZE, NoChange );



	sigma.block( ODO_SIGMA_SIZE, 0, sigma.rows() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE, sigma.cols() ) =
			sigma.block( ODO_SIGMA_SIZE + n * ODO_SIGMA_FRAME_SIZE, 0, sigma.rows() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE, sigma.cols() );
	sigma.block( 0, ODO_SIGMA_SIZE, sigma.rows(), sigma.cols() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE ) =
			sigma.block( 0, ODO_SIGMA_SIZE + n * ODO_SIGMA_FRAME_SIZE, sigma.rows(), sigma.cols() - ODO_SIGMA_SIZE - n * ODO_SIGMA_FRAME_SIZE );
	sigma.conservativeResize( sigma.rows() - n * ODO_SIGMA_FRAME_SIZE, sigma.cols() - n * ODO_SIGMA_FRAME_SIZE );
	
}


//
// get the most likely 3d position of a feature
// 
// z is the observations of a feature
//
Vector3d MSCKF::triangulate( MatrixX2d z ) {
	unsigned int itterations = 3;

	//
	// Get estimate of world feature for each measurement
	//
	MatrixX3d G_p_fi( z.rows(), 3 );
	bool clearOutlier = false;
	for( int i = 0; i < z.rows(); i ++ ) {
		//
		// undistort r to get beta = (u,v)
		//
		// initial guess:
		Vector2d beta( 
			( z( i, 0 ) - calib->o_x ) / calib->f_x,
			( z( i, 1 ) - calib->o_y ) / calib->f_y
		);

		// itterate:
		for( int j = 0; j < itterations; j++ ) {
			// Residual
			Vector2d r = z.row( i ).transpose() - cameraProject( beta(0), beta(1), 1, calib );
			// Jacobian
			Matrix2d Jf = jacobianH( beta(0), beta(1), 1, calib ).block<2,2>(0,0);
			// New estimate
			beta = beta + (Jf.transpose()*Jf).inverse() * Jf.transpose() * r;
		}

		//
		// calculate camera position:
		//
		// Get index of start of this frame in state
		unsigned int frameStart = x.rows() - ODO_STATE_FRAME_SIZE*( z.rows() - i + 1 );
		// Get inertial frame state at that time:	
		Quaternion<double> IiG_q( x.block<4,1>( frameStart + 0, 0 ) );
		Quaternion<double> CiG_q = calib->CI_q * IiG_q;
		// Calculate camera state
		Vector3d G_p_Ii = x.block<3,1>( frameStart + 4, 0 );
		Vector3d G_p_Ci = G_p_Ii - CiG_q.conjugate()._transformVector( calib->C_p_I );

		// Calculate feature position estimate
		Vector3d Ci_theta_i( beta(0), beta(1), 1 );
		Vector3d G_theta_i = CiG_q.conjugate()._transformVector( Ci_theta_i );
		double t_i = - G_p_Ci( 2 ) / G_theta_i( 2 );
		G_p_fi.row( i ) = ( t_i * G_theta_i + G_p_Ci ).transpose();

		// Check if feature position is estimated behind camera
		if ( t_i<= 0 )
			clearOutlier = true;
	}

	//
	// Average estimates
	//
	Vector3d G_p_f( G_p_fi.col(0).mean(), G_p_fi.col(1).mean(), G_p_fi.col(2).mean() );

	std::cout << " G_p_f: " << G_p_f.transpose();
	if ( clearOutlier )
		std::cout << "    clearOutlier" << std::endl;
	else
		std::cout << "    not clearOutlier" << std::endl;


	//
	// Signal no solution
	//
	if ( !std::isfinite(G_p_f(0)) || !std::isfinite(G_p_f(0)) || !std::isfinite(G_p_f(0)) || clearOutlier )
		G_p_f = Vector3d( NAN, NAN, NAN );

	return G_p_f;
}

//
// Marginalize
// From a list of feature measurements z, and the estimated feature position G_p_f
// calculate the residuals (r0) and state jacobian (H0) marginalizing out the feature error
// A block matrix can be parsed to r0 and H0, if this is of the correct size
//
void MSCKF::marginalize( const MatrixX2d &z , const Vector3d &G_p_f, Ref<VectorXd> r0, Ref<MatrixXd> H0 ) {
	//
	// calculate residuals and 
	//
	VectorXd r( z.rows()*2 );
	MatrixX3d H_f( z.rows()*2 ,3 );
	MatrixXd H_x = MatrixXd::Zero( z.rows()*2, sigma.cols() );

	for( int i = 0; i < z.rows(); i ++ ) {
		//
		// calculate camera position:
		//
		// Get index of start of this frame in state
		unsigned int frameStart = x.rows() - ODO_STATE_FRAME_SIZE*( z.rows() - i + 1 );
		// Get inertial frame state at that time:	
		Quaternion<double> IiG_q( x.block<4,1>( frameStart + 0, 0 ) );
		Quaternion<double> CiG_q = calib->CI_q * IiG_q;
		// Calculate camera state
		Vector3d G_p_Ii = x.block<3,1>( frameStart + 4, 0 );
		Vector3d G_p_Ci = G_p_Ii - CiG_q.conjugate()._transformVector( calib->C_p_I );

		// Calculate feature position in camera frame
		Vector3d C_p_f = CiG_q._transformVector( G_p_f - G_p_Ci );

		r.block<2,1>( i*2, 0 ) = z.row( i ).transpose() - cameraProject( C_p_f(0), C_p_f(1), C_p_f(2), calib );
		H_f.block<2,3>( i*2,0 ) = jacobianH( C_p_f(0), C_p_f(1), C_p_f(2), calib ) * CiG_q.toRotationMatrix();
		H_x.block<2,9>( i*2, H_x.cols() - ODO_SIGMA_FRAME_SIZE*( z.rows() - i + 1 ) ) <<
				H_f.block<2,3>( i*2,0 )*crossMat( G_p_f - G_p_Ii ), -H_f.block<2,3>( i*2,0 ), Matrix<double,2,3>::Zero();
	}

	// Find left null-space
	Eigen::FullPivLU<MatrixXd> LU( H_f.transpose() );
	MatrixXd A = LU.kernel().transpose();

	if ( z.rows()*2 - 3 != A.rows() ) {
		std::cout << "A: " << A.rows() << "x" << A.cols() << std::endl;
		std::cout << "z: " << z.rows() << "x" << z.cols() << std::endl;
		std::cout << "r0: " << r0.rows() << "x" << r0.cols() << std::endl;
		std::cout << "H0: " << H0.rows() << "x" << H0.cols() << std::endl;
		std::cout << "H_f: " << H_f.rows() << "x" << H_f.cols() << std::endl;
		std::cout << "H_x: " << H_x.rows() << "x" << H_x.cols() << std::endl;
		std::cout << "H_f" << H_f << std::endl;
		std::cout << "H_x" << H_x << std::endl;
	}

	// Marginalize
	//r0.resize( A.rows(), NoChange );
	//H0.resize( A.rows(), sigma.cols() );
	r0 = A * r;
	H0 = A * H_x;

}

void MSCKF::updateCamera( CameraMeasurements &cameraMeasurements ) {
	//
	// Append current state to frame FIFO
	//
	this->augmentState( );

	// initialize H0 and r0
	VectorXd r0( 0 );
	MatrixXd H0( 0, sigma.cols() );
	//
	// Get max length of "living" feature r0 and H0
	//
	unsigned int longestLiving = 0;
	for ( std::list<CameraMeas_t>::iterator meas_j = cameraMeasurements.meas.begin(); meas_j != cameraMeasurements.meas.end(); ) {
		// Enforce maximum age of features
		if ( meas_j->z.rows() >= calib->maxFrame ) {
			meas_j->isLost = true;
			meas_j->z.conservativeResize( z.rows()-1, NoChange );
		}
		if ( meas_j->isLost ) {
			// If more that, or 3 points, use for update
			if ( meas_j->z.rows() >= 3 ) {
				Vector3d G_p_fj = this->triangulate( meas_j->z );
				// If not a clear outlier:
				if ( std::isfinite(G_p_fj(0)) && std::isfinite(G_p_fj(1)) && std::isfinite(G_p_fj(2)) ) {
					// Marignalize:
					VectorXd r0j = VectorXd( meas_j->z.rows()*2-3 );
					MatrixXd H0j = MatrixXd( meas_j->z.rows()*2-3, sigma.cols() );
					this->marginalize( meas_j->z, G_p_fj, r0j, H0j );
					// TODO: Check if inlier
					if ( isInlinerCamera( r0j, H0j ) ) {
						// debug draw
						Eigen::MatrixX2d& z = meas_j->z;
						cv::Point pt = cv::Point( z( z.rows()-1, 0 ), z( z.rows()-1, 1 ) );
						cv::circle( debugImg, pt, 4, cv::Scalar( 0, 255, 0 ) );
						for( int i = 0; i < z.rows() - 1; i++ ) {
							cv::Point pt1 = cv::Point( z( i, 0 ), z( i, 1 ) );
							cv::Point pt2 = cv::Point( z( i+1, 0 ), z( i+1, 1 ) );
							cv::line( debugImg, pt1, pt2, cv::Scalar( 0, 255, 0 ) );
						}

						// Add to huge H0 and r0 matrix
						H0.conservativeResize( H0j.rows(), NoChange );
						r0.conservativeResize( r0j.rows(), NoChange );
						H0.bottomRows( H0j.rows() ) = H0j;
						r0.bottomRows( r0j.rows() ) = r0j;
					} else {
						// debug draw
						Eigen::MatrixX2d& z = meas_j->z;
						cv::Point pt = cv::Point( z( z.rows()-1, 0 ), z( z.rows()-1, 1 ) );
						cv::circle( debugImg, pt, 4, cv::Scalar( 0, 0, 255 ) );
						for( int i = 0; i < z.rows() - 1; i++ ) {
							cv::Point pt1 = cv::Point( z( i, 0 ), z( i, 1 ) );
							cv::Point pt2 = cv::Point( z( i+1, 0 ), z( i+1, 1 ) );
							cv::line( debugImg, pt1, pt2, cv::Scalar( 0, 0, 255 ) );
						}
					}
				} else {
					// debug draw
					Eigen::MatrixX2d& z = meas_j->z;
					cv::Point pt = cv::Point( z( z.rows()-1, 0 ), z( z.rows()-1, 1 ) );
					cv::circle( debugImg, pt, 4, cv::Scalar( 0, 255, 255 ) );
					for( int i = 0; i < z.rows() - 1; i++ ) {
						cv::Point pt1 = cv::Point( z( i, 0 ), z( i, 1 ) );
						cv::Point pt2 = cv::Point( z( i+1, 0 ), z( i+1, 1 ) );
						cv::line( debugImg, pt1, pt2, cv::Scalar( 0, 255, 255 ) );
					}
				}
			}
			// in any case, remove it and advance
			meas_j = cameraMeasurements.removeFeature( meas_j );
		} else {
			// Set longest living
			longestLiving = (meas_j->z.rows()>longestLiving)?meas_j->z.rows():longestLiving;
			// Skip and advance
			++meas_j;
		}
	}
	//
	// TODO: QR decomposition (skipped for now)
	//

	//
	// Calculate kalmangain and apply update
	//
	// Only if we have measurements
	if ( r0.rows() > 0 ) {
		// Image noise
		MatrixXd R_q = MatrixXd::Identity( r0.rows(), r0.rows() ) * calib->sigma_Im;

		// Kalman gain
		MatrixXd K = sigma * H0.transpose() * ( H0 * sigma * H0.transpose() + R_q ).inverse();

		// Update to be appled to state
		VectorXd delta_x = K * r0;

		// Update covariance
		MatrixXd A = MatrixXd::Identity( K.rows(), H0.cols() ) - K * H0;
		sigma = A * sigma * A.transpose() + K * R_q * K.transpose();

		//
		// apply feedback
		//

		// To inertial state
		// IG_q
		Quaternion<double> delta_IG_q( 1, delta_x(0), delta_x(1), delta_x(2) );
		Quaternion<double> IG_q = ( Quaternion<double>( x.block<4,1>( 0, 0 ) ) * delta_IG_q );
		IG_q.normalize();
		x.block<4,1>( 0, 0 ) = IG_q.coeffs();
		// G_p
		x.block<3,1>( 0+4, 0 ) += delta_x.block<3,1>( 0+3, 0 );
		// G_v
		x.block<3,1>( 0+4+3, 0 ) += delta_x.block<3,1>( 0+3+3, 0 );
		// b_g
		x.block<3,1>( 0+4+3+3, 0 ) += delta_x.block<3,1>( 0+3+3+3, 0 );
		// b_a
		x.block<3,1>( 0+4+3+3+3, 0 ) += delta_x.block<3,1>( 0+3+3+3+3, 0 );

		// to all the frames
		for ( int i = 0; i < ( x.rows() - ODO_STATE_SIZE ) / ODO_STATE_FRAME_SIZE; i++ ) {
			unsigned int frameStart = ODO_STATE_SIZE + i * ODO_STATE_FRAME_SIZE;
			unsigned int delta_frameStart = ODO_SIGMA_SIZE + i * ODO_SIGMA_FRAME_SIZE;
			Quaternion<double> delta_IiG_q( 1,
					delta_x( delta_frameStart + 0 ),
					delta_x( delta_frameStart + 1 ),
					delta_x( delta_frameStart + 2 )
			);
			Quaternion<double> IiG_q = ( Quaternion<double>( x.block<4,1>( frameStart + 0, 0 ) ) * delta_IiG_q );
			IiG_q.normalize();
			x.block<4,1>( frameStart + 0, 0 ) = IiG_q.coeffs();
			// G_p_i
			x.block<3,1>( frameStart + 0+4, 0 ) += delta_x.block<3,1>( delta_frameStart + 0+3, 0 );
			// G_v_i
			x.block<3,1>( frameStart + 0+4+3, 0 ) += delta_x.block<3,1>( delta_frameStart + 0+3+3, 0 );
		}
	}

	//
	// Remove all old and unused frames
	//

	//
	// Make sure that we let atleast "minFrame" frames live
	//
	if ( longestLiving < calib->minFrame )
		longestLiving = calib->minFrame;
	this->removeOldStates( ( x.rows() - ODO_STATE_SIZE ) / ODO_STATE_FRAME_SIZE - longestLiving );

}

void MSCKF::updateHeight( double height ) {
	//
	// TODO: check if outlier
	//

	//
	// TODO: Optimize since H is very sparse (only one element)
	//

	// Sensor noise
	Matrix<double,1,1> R;
	R << calib->sigma_hc; // cause we need to add this to another one
	// Sensor residual
	double r = height - x(0+4+2); // x(0+4+3) is G_p(2)
	// Sensor model
	Matrix<double,1,Dynamic> H( 1, sigma.cols() );
	H << MatrixXd::Zero( 1, 5 ), 1, MatrixXd::Zero( 1, sigma.cols() - 6 );

	// Kalman gain
	MatrixXd K = sigma * H.transpose() * ( H * sigma * H.transpose() + R ).inverse();

	// Update to be appled to sf4ate
	VectorXd delta_x = K * r;

	// Update covariance
	MatrixXd A = MatrixXd::Identity( K.rows(), H.cols() ) - K * H;
	sigma = A * sigma * A.transpose() + K * R * K.transpose();

	//
	// apply feedback
	//

	// To inertial state
	// IG_q
	Quaternion<double> delta_IG_q( 1, delta_x(0), delta_x(1), delta_x(2) );
	Quaternion<double> IG_q = ( Quaternion<double>( x.block<4,1>( 0, 0 ) ) * delta_IG_q );
	IG_q.normalize();
	x.block<4,1>( 0, 0 ) = IG_q.coeffs();
	// G_p
	x.block<3,1>( 0+4, 0 ) += delta_x.block<3,1>( 0+3, 0 );
	// G_v
	x.block<3,1>( 0+4+3, 0 ) += delta_x.block<3,1>( 0+3+3, 0 );
	// b_g
	x.block<3,1>( 0+4+3+3, 0 ) += delta_x.block<3,1>( 0+3+3+3, 0 );
	// b_a
	x.block<3,1>( 0+4+3+3+3, 0 ) += delta_x.block<3,1>( 0+3+3+3+3, 0 );

	// to all the frames
	for ( int i = 0; i < ( x.rows() - ODO_STATE_SIZE ) / ODO_STATE_FRAME_SIZE; i++ ) {
		unsigned int frameStart = ODO_STATE_SIZE + i * ODO_STATE_FRAME_SIZE;
		unsigned int delta_frameStart = ODO_SIGMA_SIZE + i * ODO_SIGMA_FRAME_SIZE;
		Quaternion<double> delta_IiG_q( 1,
				delta_x( delta_frameStart + 0 ),
				delta_x( delta_frameStart + 1 ),
				delta_x( delta_frameStart + 2 )
		);
		Quaternion<double> IiG_q = ( Quaternion<double>( x.block<4,1>( frameStart + 0, 0 ) ) * delta_IiG_q );
		IiG_q.normalize();
		x.block<4,1>( frameStart + 0, 0 ) = IiG_q.coeffs();
		// G_p_i
		x.block<3,1>( frameStart + 0+4, 0 ) += delta_x.block<3,1>( delta_frameStart + 0+3, 0 );
		// G_v_i
		x.block<3,1>( frameStart + 0+4+3, 0 ) += delta_x.block<3,1>( delta_frameStart + 0+3+3, 0 );
	}

}

bool MSCKF::isInlinerCamera( const VectorXd &r0, const MatrixXd &H0 ) {
	double gamma = r0.transpose() * ( H0 * sigma * H0.transpose() ).inverse() * r0;
	return gamma <= chi2Inv[ r0.rows() ];
}