#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_


#define ODO_MAX_FRAMES (5) // used for static allocation of state and sigma
#define ODO_STATE_SIZE (16)
#define ODO_STATE_FRAME_SIZE (10)
#define ODO_STATE_MAX_SIZE ( ODO_STATE_SIZE + ODO_STATE_FRAME_SIZE * ( ODO_MAX_FRAMES + 1 ) )
#define ODO_SIGMA_SIZE (15)
#define ODO_SIGMA_FRAME_SIZE (9)
#define ODO_SIGMA_MAX_SIZE ( ODO_SIGMA_SIZE + ODO_SIGMA_FRAME_SIZE * ( ODO_MAX_FRAMES + 1 ) )


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "quaternionAlias.hpp"
#include <sys/time.h>
#include <list>
#include "common.hpp"

using namespace Eigen;


class Calib {
public:
	//
	// Camera calibration
	//
	/* Projection */
	double o_x, o_y,  // principal point [px,px]
			f_x, f_y, // focal length [px,px]
			k1, k2,   // radial distortion parameters [n/u,n/u]
			t1, t2;   // tangential distortion parameters [n/u,n/u]
	/* Position */
	QuaternionAlias<double> CI_q; // Rotation from intertial to camera coordinates. [unit quaternion]
	Vector3d C_p_I;              // Position of inertial frame in camera coordinates [m,m,m]
	//
	// Physical properties
	//
	double g;           // Gravitational acceleration [m/s^2]
	double delta_t;     // Time between IMU data [s]
	struct timeval imageOffset; // Time delay for images [s]
	//
	// Noise levels
	//
	/* IMU TODO: add correct units */
	double sigma_gc, sigma_ac,    // Standard deviation of IMU noise [rad/s, m/s^2]
			sigma_wgc, sigma_wac; // Standard deviation of IMU random walk noise [rad/s, m/s^2]
	/* Camera */
	double sigma_Im; // Image noise
	/* height sensor */
	double sigma_hc; // Standard deviation of height sensor noise [m]
	//
	// Options
	//
	unsigned int maxFrame; // Maximum frames in FIFO
	unsigned int minFrame; // Minimun frames in FIFO (is smaller before the first minFrames). Is used if features are only added when there is a match

	Calib( );
	friend std::ostream& operator<<( std::ostream& out, const Calib& calib );
};

class Odometry {
public:
	//
	// Calibration object
	//
	Calib* calib;
	//
	// State
	//
	// Matrix<double,Dynamic,1,0,ODO_STATE_MAX_SIZE,1> x; // allocate statically
	Matrix<double,Dynamic,1> x;
	//
	// Covariance
	//
	// Matrix<double,Dynamic,Dynamic,0,ODO_SIGMA_MAX_SIZE,ODO_SIGMA_MAX_SIZE> sigma; // allocate statically
	Matrix<double,Dynamic,Dynamic> sigma;
	//
	// Local variables for integration
	//
	Vector3d I_a_dly, I_g_dly;

	// init
	Odometry( Calib* cal );
	// propagate
	void propagate( double a_m[3], double g_m[3], bool propagateError = true );
	// TODO: -Propagate state <- used for pure prediction
	// TODO: -Propagate sigma <- no use for this
	// augment state
	void augmentState( void );
	// remove n old states
	void removeOldStates( int n );
	// updateHeight
	void updateHeight( double height );
	// TODO: -isInlierHeight
	bool isInlinerHeight( const double r, const MatrixXd &H );
	// performUpdate
	void performUpdate( const VectorXd &delta_x );
};


class MSCKF: public Odometry {
public:
	cv::Mat debugImg;

//public:
	// print
	friend std::ostream& operator<<( std::ostream& out, const MSCKF& msckf );
	// init
	MSCKF( Calib* cal ): Odometry( cal ){};
	// updateCamera
	void updateCamera( CameraMeasurements &cameraMeasurements );
	// 	triangulate
	Vector3d triangulate( MatrixX2d z );
	// -marginalize
	void marginalize( const MatrixX2d &z, const Vector3d &G_p_f, Ref<VectorXd> r0, Ref<MatrixXd> H0 );
	// TODO: -isInlinerCamera
	bool isInlinerCamera( const VectorXd &r0, const MatrixXd &H0 );
	// Initiate by running this for some time
	void updateInit( double height );
};

//
// TODO: move to file with utility functions
//
Matrix<double,2,3> jacobianH( double X, double Y, double Z, const Calib* calib );
Vector2d cameraProject( double X, double Y, double Z, const Calib* calib );
Matrix4d Omega( const Vector3d& v );
Matrix3d crossMat( const Vector3d& v );
Vector2d featureUndistort( const Vector2d &src, const Calib *calib, unsigned int itterations = 3 );

#endif //_ODOMETRY_H_
