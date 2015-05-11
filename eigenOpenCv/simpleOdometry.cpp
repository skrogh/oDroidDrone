#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <sys/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <cmath>
#include <ctype.h>
#include <random>
#include "lkTracker.hpp"
#include "odometry.hpp"
#include "imu.hpp"
#include "videoIO.hpp"
#include "telemetry.hpp"


using namespace cv;
using namespace std;
using namespace Eigen;

void trackerThreadHandler( LKTracker* tracker, cv::Mat* gray, cv::Mat* prevGray ) {
	tracker->detectFeatures( *gray, *prevGray );
}

void catchupPredictorThread( std::atomic<bool>* catchupRunningPt, std::atomic<bool>* catchupDonePt,
ImuFifo* catchupImuFifoPt, Odometry* catchupPredictorPt ) {
	std::atomic<bool>& catchupRunning = *catchupRunningPt;
	std::atomic<bool>& catchupDone = *catchupDonePt;
	ImuFifo& catchupImuFifo = *catchupImuFifoPt;
	Odometry& catchupPredictor = *catchupPredictorPt;

	// Propagate all elements
	ImuMeas_t element;
	while( catchupImuFifo.fifoPop( element ) ) {
		catchupPredictor.propagate( element.acc, element.gyro, false );
	}

	// done
	catchupRunning = false; // Stop adding new values to fifo
	catchupDone = true; // Signal that we are done
}

void estimator( ImuFifo* imuPt, Calib* calibPt,
			std::atomic<bool>* catchupRunningPt, std::atomic<bool>* catchupDonePt,
			ImuFifo* catchupImuFifoPt, Odometry* catchupPredictorPt	) {
	ImuFifo& imu = *imuPt;
	Calib& calib = *calibPt;
	std::atomic<bool>& catchupRunning = *catchupRunningPt;
	std::atomic<bool>& catchupDone = *catchupDonePt;
	ImuFifo& catchupImuFifo = *catchupImuFifoPt;
	Odometry& catchupPredictor = *catchupPredictorPt;

	//
	// Initiate estimator
	//

	// make odometry obj and set initial conditions
	GTEKF odometry( &calib );
	// Start upright
		odometry.x.block<4,1>(0,0) << 0, 0, 0, 1; // upright
	// Start 15cm off the ground
	odometry.x.block<3,1>(4,0) << 0, 0, 0.15; // 15cm from ground

	// Set initial uncertancy
	odometry.sigma.diagonal().block<3,1>(0,0) << 0.05, 0.05, 0.05; // Attitude
	odometry.sigma.diagonal().block<3,1>(3,0) << 0, 0, 0.2;        // Position
	odometry.sigma.diagonal().block<3,1>(6,0) << 0, 0, 0;          // Velocity
	odometry.sigma.diagonal().block<3,1>(9,0) << 0.1, 0.1, 0.1;    // Gyro bias
	odometry.sigma.diagonal().block<3,1>(12,0) << 0.1, 0.1, 0.1;   // Acc bias


	//
	// Initialize data IO objects
	//
	// Log for logging state at all steps (for plotting and stuff)
	std::ofstream logFile;
	logFile.open ("log.csv");
	// Video server getter (TODO: WARNING MAX ONE CONSUMER)
	VideoIn videoIn( 0 );
	// Debug image out
	cv::Mat frame;
	namedWindow( "Features", 1 );
	// IMU timestamping
	struct timeval tv;
	struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;

	//
	// Initialize other objects
	//
	// Containers for current and previous image
	Mat gray, prevGray;
	// Feature tracker
	LKTracker tracker;
	// Counter for ignoring heigth measurements to reduce computation time
	int ignoredHeights = 0;

	//
	// Estimation loop
	//
	for(;;)
	{
		//
		// Get new image
		//
		videoIn.requestImage( frame, tv );
		// convert to gray-tone
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		// Fencepost fix, if no previous image, get one and skip ret of update
		if(prevGray.empty())
		{
			gray.copyTo(prevGray);
			// skip first image
			odometry.augmentState( );
			continue;
		}

		//
		// Start detect features in parallel with propagation
		//
		sdt:thread trackerThread( trackerThreadHandler, &tracker, &gray, &prevGray );

		//
		// Propagate up to new image ( can be run in parallel with feature detection)
		//
		while( 1 ) {
			ImuMeas_t element;
			// Wait for at least one imu measurement
			imu.waitNotEmpty();
			// get it
			imu.fifoPop( element );

			// Propagate
			odometry.propagate( element.acc, element.gyro );

			// log to file
			logFile << odometry.x.block<16,1>(0,0).transpose() << "\t";
			logFile << odometry.sigma.diagonal().block<15,1>(0,0).transpose() << "\t";
			logFile << odometry.sigma.determinant() << "\t";
			logFile << odometry.sigma.diagonal().mean() << "\t";
			logFile << ( odometry.sigma - odometry.sigma.transpose() ).sum() << "\n";

			// If valid distance measurement, update with that
			if ( element.distValid )
			{
				if ( ignoredHeights >= 0 )
				{
					odometry.updateHeight( element.dist );
					ignoredHeights = 0;
				}
				else
				{
					ignoredHeights++;
				}
			}

			// Get time of image without delay
			struct timeval imageTime;
			timersub( &tv, &(calib.imageOffset), &imageTime );

			// If image is older that propagated point, stop propagation and update
			if ( timercmp( &imageTime, &(element.timeStamp), < ) ) {
 				break;
			}
		}

		// end detect features (wait for completion)
		trackerThread.join();

		//
		// Update from camera
		//

		//
		// Debug print of tracked features
		//
		for( int i = 0; i < tracker.points.size(); i++ )
		{
			line( frame, tracker.points[i], tracker.prevPoints[i], Scalar(0,255,0) );
			circle( frame, tracker.points[i], 2, Scalar(0,255,0) );
		}

		//
		// undistort points
		//

		// Initialize
		Matrix2Xd points(2, tracker.points.size());
		Matrix2Xd prevPoints(2, tracker.points.size());

		const Calib* calib = odometry.calib;
		VectorXd &x = odometry.x;
		MatrixXd &sigma = odometry.sigma;

		// Undistort
		for ( int i = 0; i < points.cols(); i++ )
		{
			points.col(i) = featureUndistort( Vector2d( tracker.points[i].x, tracker.points[i].y ), calib);
			prevPoints.col(i) = featureUndistort( Vector2d( tracker.prevPoints[i].x, tracker.prevPoints[i].y ), calib);
		}

		odometry.updateCamera( points, prevPoints, frame );

		//
		// Set up predictor catchup
		//
		/* copy estimated state to predictor catchup */
		catchupPredictor.x = odometry.x.block<ODO_STATE_SIZE,1>(0,0);
		// dont copy covariance, we dont care about that
		catchupPredictor.I_a_dly = odometry.I_a_dly;
		catchupPredictor.G_a_dly = odometry.G_a_dly;
		/* Copy rest of etimatorImuFifo to catchupImuFifo */
		imu.fifoCloneTo( catchupImuFifo );
		/* Tell main thread to add new measurements to catchupImuFifo (catchupRunning) */
		catchupRunningPt = true;
		/* Start and detach catchup thread */
		std::thread catchupThread( catchupPredictorThread,
			&catchupRunning, &catchupDone, &catchupImuFifo, &catchupPredictor );
		catchupThread.detach();

		//
		// Window managing
		//
		imshow("Features", frame);
		waitKey(1);

		//
		// Move current image to old one
		//
		cv::swap(prevGray, gray);
	}
	logFile.close();
}

void initCalib( Calib& calib ) {
	// Set calibration parameters:
	calib.o_x = 300.8859;
	calib.o_y = 222.5206;
	calib.f_x = 411.1170;
	calib.f_y = 409.9516;
	calib.k1 = -0.3453;
	calib.k2 = 0.1012;
	calib.t1 = -0.0003;
	calib.t2 = 0.0014;
	calib.CI_q = Eigen::QuaternionAlias<double>(
			 0.000000000000000,
			 0.707106781186548,
			 0.707106781186548,
			 0.000000000000000
	);
	calib.C_p_I = Eigen::Vector3d( 0.0, -0.044, -0.037 );
	calib.g = 9.82;
	calib.delta_t = 0.0025;
	calib.imageOffset.tv_sec = 0;
	calib.imageOffset.tv_usec = 33000 + 7000; // delay of 1 frame period + some
	calib.sigma_gc = 0.001;//5.0e-04;
	calib.sigma_ac = 0.008;//5.0e-04;
	calib.sigma_wgc = 0.0001;
	calib.sigma_wac = 0.0001;
	calib.sigma_Im = 40;
	calib.sigma_hc = 0.05;
	calib.minFrame = 1;
	std::cout << "calib is:\n" << calib << std::endl;
}

int main( int argc, char** argv )
{
	// Set calibration parameters:
	Calib calib;
	initCalib( calib );

	/* TODO: re-add telemetry
	Telemetry telemetry( 55000 );
	// log over telemetry
	if ( telemetryCounter++ > 40 ) {
		telemetryCounter = 0;
		telemetry.send( odometry.x.data(), sizeof(double)*10 ); // send quaternion, position and velocity
	}
	int telemetryCounter = 0;
	*/


	Imu imu( "/dev/spidev1.0", "/sys/class/gpio/gpio199/value" );
	struct timeval tv;
	struct timezone tz = {};
		tz.tz_minuteswest = 0;
		tz.tz_dsttime = 0;
	//
	//	imu data buffers for estimation and propagation
	//
	ImuFifo estimatorImu;

	//
	// Clear Imu buffer
	//
	ImuMeas_t element;
	while( imu.fifoPop( element ) );

	//
	// Predictor
	//
	Odometry predictor( &calib );

	//
	// Predictor catchup
	//
	std::atomic<bool> catchupRunning(false);
	std::atomic<bool> catchupDone(false);
	ImuFifo catchupImuFifo;
	Odometry catchupPredictor( &calib );


	//
	// Start estimator
	//
	std::thread estimatorThread( estimator, &estimatorImu, &calib,
	 &catchupRunning, &catchupDone, &catchupImuFifo, &catchupPredictor );

	//
	// Start controller loop
	//
	while(1) {
		imu.waitNotEmpty(); // wait for next sample
		ImuMeas_t element;
		imu.fifoPop( element );
		// pass element to estimator
		estimatorImu.fifoPush( element );

		// If new predictor caught up, replace current with that one
		if ( catchupDone ) {
			catchupDone = false;
			predictor = catchupPredictor;
		}

		// pass element to catchup if running
		if ( catchupRunning )
			catchupImuFifo.fifoPush( element );
		// predict
		predictor.propagate( element.acc, element.gyro, false );
		// controller goes here
	}



	estimatorThread.join();


}
