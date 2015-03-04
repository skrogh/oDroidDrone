#!/bin/bash

echo "Clearing log files"
#Clear images
echo "Removing images:"
[ "$(ls -A opencv_test_images)" ] && rm opencv_test_images/* || echo "No images to clear"

#Clear log
echo "Clearing log"
[ -e log.csv ] && rm log.csv || echo "No log to clear"

