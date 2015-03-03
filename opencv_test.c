#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;

int main()
{
    VideoCapture cap(-1);     // get 'any' cam
    while( cap.isOpened() )   // check if we succeeded
    {
        Mat frame;
        if ( ! cap.read(frame) )
            break;
        imshow("lalala",frame);
        int k = waitKey(33);
        if ( k==27 )
            break;
    }
    return 0;
}