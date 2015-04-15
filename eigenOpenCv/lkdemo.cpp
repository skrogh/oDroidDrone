#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <random>
#include "lkTracker.hpp"


using namespace cv;
using namespace std;

static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
			"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
			"\tESC - quit the program\n"
			"\tr - auto-initialize tracking\n"
			"\tc - delete all the points\n"
			"\tn - switch the \"night\" mode on/off\n"
			"To add/remove a feature point click it\n" << endl;
}

int main( int argc, char** argv )
{
	help();

	VideoCapture cap;

	if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		cap.open(argc == 2 ? argv[1][0] - '0' : 0);
	else if( argc == 2 )
		cap.open(argv[1]);

	if( !cap.isOpened() )
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}
	// size
	cap.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

	namedWindow( "LK Demo", 1 );
	setMouseCallback( "LK Demo", onMouse, 0 );

	Mat gray, prevGray;
	LKTracker tracker;

	for(;;)
	{
		Mat frame;
		cap.grab();
		cap.retrieve(frame);
		if( frame.empty() )
			break;

		cvtColor(frame, gray, COLOR_BGR2GRAY);
		if(prevGray.empty())
			gray.copyTo(prevGray);

		tracker.detectFeatures( gray, prevGray );

		for( int i = 0; i < tracker.points.size(); i++ )
		{
			line( frame, tracker.points[i], tracker.prevPoints[i], Scalar(0,255,0) );
			circle( frame, tracker.points[i], 2, Scalar(0,255,0) );
		}
		imshow("LK Demo", frame);

		char c = (char)waitKey(1);
		if( c == 27 )
			break;
		switch( c )
		{
		case 'c':
			tracker.prevPoints.clear();
			tracker.points.clear();
			break;
		}

		cv::swap(prevGray, gray);
	}

	return 0;
}