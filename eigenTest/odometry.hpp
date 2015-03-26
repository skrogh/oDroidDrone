#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_


#define ODO_MAX_FRAMES (10) // used for static allocation of state and sigma
#define ODO_STATE_SIZE (16)
#define ODO_STATE_FRAME_SIZE (10)
#define ODO_STATE_MAX_SIZE ( ODO_STATE_SIZE + ODO_STATE_FRAME_SIZE * ( ODO_MAX_FRAMES + 1 ) )
#define ODO_SIGMA_SIZE (15)
#define ODO_SIGMA_FRAME_SIZE (9)
#define ODO_SIGMA_MAX_SIZE ( ODO_SIGMA_SIZE + ODO_SIGMA_FRAME_SIZE * ( ODO_MAX_FRAMES + 1 ) )


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>

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
	Quaternion<double> CI_q; // Rotation from intertial to camera coordinates. [unit quaternion]
	Vector3d C_p_I;              // Position of inertial frame in camera coordinates [m,m,m]
	//
	// Physical properties
	//
	double g;           // Gravitational acceleration [m/s^2]
	double delta_t;     // Time between IMU data [s]
	double imageOffset; // Time delay for images [s]
	//
	// Noise levels
	//
	/* IMU TODO: add correct units */
	double sigma_gc, sigma_ac,    // Standard deviation of IMU noise [rad/s, m/s^2]
			sigma_wgc, sigma_wac; // Standard deviation of IMU random walk noise [rad/s, m/s^2]
	/* Camera */
	double sigma_Im; // Image noise
	/* distance sensor */
	double sigma_dc; // Standard deviation of distance sensor noise [m]
	//
	// Options
	//
	unsigned int maxFrame; // Maximum frames in FIFO

	Calib( );
	friend std::ostream& operator<<( std::ostream& out, const Calib& calib );
};

typedef struct{
	MatrixX2d z; // length is stored as number of elements in z
	bool isLost; 
} CameraMeas_t;

class MSCKF {
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

//public:
	// print
	friend std::ostream& operator<<( std::ostream& out, const MSCKF& msckf );
	// init
	MSCKF( Calib* cal );
	// propagate
	void propagate( double a_m[3], double g_m[3] );
	// 	Propagate state <- used for pure prediction
	// 	Propagate sigma <- no use for this
	// augment state
	void augmentState( void );
	// remove n old states
	void removeOldStates( unsigned int n );
	// updateCamera
	void updateCamera( std::list<CameraMeas_t> meas );
	// 	triangluate
	Vector3d triangluate( MatrixX2d z );
	// -marginalize
	void marginalize( MatrixX2d z, Vector3d G_p_f, Ref<VectorXd> r0, Ref<MatrixXd> H0 );
	// -isInlinerCamera
	// updateDistance
	// -isInlierDistance
};


#endif //_ODOMETRY_H_