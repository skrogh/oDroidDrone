#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>

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

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}

int main( int argc, char** argv )
{
    help();

    VideoCapture cap;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 200;
    const int N_OPTIMAL = 20;
    bool needToInit = false;
    bool nightMode = false;

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
    vector<Point2f> points, prevPoints;

    for(;;)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        /*
            // automatic initialization
            Mat roi(gray, Rect(0,0,100,100));
            goodFeaturesToTrack(roi, points, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            //cornerSubPix(gray, points, subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        */
        // Check if we need to add more points
        if ( prevPoints.size() < N_OPTIMAL ) {
            // get region of interest
            int roiX = 0;
            int roiY = 0;
            Point2f roiP( roiX, roiY );
            Mat roi(gray, Rect(roiX,roiY,100,100));

            vector<Point2f> gfttPoints;
            goodFeaturesToTrack(roi, gfttPoints, N_OPTIMAL-prevPoints.size(), 0.01, 10, Mat(), 3, 0, 0.04);
            for ( int i = 0; i < gfttPoints.size(); i++ ) {
                prevPoints.push(gfttPoints[i] + roiP);
            }
        }

        if( !prevPoints.empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, prevPoints, points, status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points.size(); i++ )
            {
                //´Skip this point, if it is invalid
                if( !status[i] )
                    continue;

                // Add it if it is valid (since i>k it is safe to use the poins vector for this)
                points[k++] = points[i];
                circle( image, points[i], 3, Scalar(0,255,0), -1, 8);
            }
            // k is the number of valid points
            points.resize(k);
        }



        imshow("LK Demo", gray);

        char c = (char)waitKey(1);
        if( c == 27 )
            break;
        switch( c )
        {
        case 'c':
            prevPoints.clear();
            points.clear();
            break;
        }

        std::swap(points, prevPoints);
        cv::swap(prevGray, gray);
    }

    return 0;
}