#!/bin/bash

echo "Clearing log files"
#Clear images
echo "Removing images:"
[ "$(ls -A opencvImageGrab/opencv_test_images)" ] && rm opencvImageGrab/pencv_test_images/* || echo "No images to clear"

#Clear log
echo "Clearing log"
[ -e imuGrab/log.csv ] && rm imuGrab/log.csv || echo "No log to clear"

