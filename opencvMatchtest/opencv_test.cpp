#include <linux/videodev2.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace cv;

int main()
{
	printf( "Started\n" );
	// Open V4L2 device
	int  v4l2_device = open( "/dev/video0", O_RDWR );
	if ( v4l2_device < 0 ) {
		perror( "Opening device:" );
		return -1;
	}

	// Make control structure 
	struct v4l2_control ctrl;
	// and set some controls (more to be added here)
	ctrl.id = V4L2_CID_CHROMA_GAIN;
	ctrl.value = 3;
	ioctl( v4l2_device, VIDIOC_S_CTRL, &ctrl );

	// alternatively set parameters like this:
	system( "v4l2-ctl --set-fmt-video=width=380,height=240" );
	system( "v4l2-ctl --set-parm=125" );

	// get capabilities:
	struct v4l2_capability v4l2_caps = {0};
	if ( ioctl( v4l2_device, VIDIOC_QUERYCAP, &v4l2_caps ) < 0 ) {
		perror( "Querying Capabilites:" );
		return -1;
	}

	// set format (did that already)

	// request buffer
	struct v4l2_requestbuffers v4l2_req = {0};
	v4l2_req.count = 1;
	v4l2_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v4l2_req.memory = V4L2_MEMORY_MMAP;
	 
	if ( ioctl( v4l2_device, VIDIOC_REQBUFS, &v4l2_req ) < 0 ) {
		perror( "Requesting Buffer:" );
		return -1;
	}

	// query buffer
	struct v4l2_buffer v4l2_buf = {0};
	v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v4l2_buf.memory = V4L2_MEMORY_MMAP;
	v4l2_buf.index = 0;
	v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if( ioctl( v4l2_device, VIDIOC_QUERYBUF, &v4l2_buf ) < 0) {
		perror( "Querying Buffer" );
		return 1;
	}

	printf( "Type %d\n", v4l2_buf.type );

	uint8_t *buffer = (uint8_t*)mmap ( NULL, v4l2_buf.length,
		PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_device, v4l2_buf.m.offset );


	Mat frame;

	// start capture
	if( ioctl( v4l2_device, VIDIOC_STREAMON, &v4l2_buf.type ) < 0 ) {
		perror( "Start Capture:" );
		return -1;
	}

	while( 1 ) {
		// wait for frame
		fd_set fds;
		FD_ZERO( &fds );
		FD_SET( v4l2_device, &fds );
		struct timeval timeout = {0};
		timeout.tv_sec = 2;
		int r = select( v4l2_device+1, &fds, NULL, NULL, &timeout );
		if( r < 1 ) {
			perror( "Waiting for Frame:" );
			return -1;
		}
		// Grab it
		if( ioctl( v4l2_device, VIDIOC_DQBUF, &v4l2_buf ) < 0) {
			perror( "Retrieving Frame:" );
			return -1;
		}

		// show image
		imshow( "edges", frame );
		// wait a while (gives CV time to show image, and desk to save image)
		if( waitKey( 60 ) >= 0 ) break;
	}
	return 0;
}
