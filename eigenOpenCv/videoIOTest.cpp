#include "videoIO.hpp"

int main( int argc, char** argv )
{
	VideoIn videoIn( 0 );
	cv::Mat frame;

	namedWindow( "Features", 1 );

	while(1)
	{
		imshow("Features", frame);

		char c = (char)waitKey(1);
		if( c == 27 )
			break;
	}
}