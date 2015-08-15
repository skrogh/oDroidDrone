extern "C" {
  #include "encoder.h"
}
#include "opencv2/opencv.hpp"

#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[])
{
	struct options opts;

	std::cout << "mfc codec encoding example application" << std::endl <<
	       "Original by Andrzej Hajda <a.hajda@samsung.com>" << std::endl <<
	       "Copyright 2012 Samsung Electronics Co., Ltd." << std::endl <<
         "openCV interface by Søren Andersen." << std::endl <<
         "Copyright 2015" << std::endl << std::endl;

  char *inBuff[2];

  if (parse_args(&opts, "/dev/video9", "video.avi", 640, 480, "h264", inBuff)) {
    return 1;
  }

  /* open capture dev */
  cv::VideoCapture capDev;
  if (!capDev.open(0))
    return -1;
  capDev.set(cv::CAP_PROP_FRAME_HEIGHT, /*HEIGHT*/ 640);
  capDev.set(cv::CAP_PROP_FRAME_WIDTH, /*WIDTH*/ 480);

  /* grab popcor... image */
  cv::Mat frame;
  capDev.read(frame);
  cv::Mat YcrCb;
  cv::Mat Gray(frame.rows, frame.cols, CV_8UC1);
  cv::Mat CbCr(frame.rows,frame.cols, CV_8UC2);
  cv::Mat CbCr_2(frame.rows/2, frame.cols/2, CV_8UC2);
  cv::cvtColor(frame, YcrCb, COLOR_BGR2YCrCb);

  cv::Mat out[] = {Gray, CbCr};
  int from_to[] = { 0,0, 2,1, 1,2 };
  cv::mixChannels(&YcrCb, 1, out, 2, from_to, 3);

  cv::resize(CbCr, CbCr_2, cv::Size(), 0.5, 0.5, INTER_NEAREST);
  inBuff[0] = (char*) Gray.data;
  inBuff[1] = (char*) CbCr_2.data;

  char a[11] = "HelloWorld";
	write( opts.encoderFd, a, 10 );

	return encoderStart( &opts );
}
