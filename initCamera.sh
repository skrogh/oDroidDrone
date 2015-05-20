#!/bin/bash
# must be root
# run with: su -c "sh initGpio.sh"

# Set camera parameters through v4l2-ctl
echo 'Setting camera parameters'
v4l2-ctl --set-ctrl=<ctl>=<val>
v4l2-ctl --set-ctrl=brightness=0
v4l2-ctl --set-ctrl=contrast=32
v4l2-ctl --set-ctrl=saturation=64
v4l2-ctl --set-ctrl=hue=0
v4l2-ctl --set-ctrl=white_balance_automatic=0
v4l2-ctl --set-ctrl=gain_automatic=1
v4l2-ctl --set-ctrl=auto_exposure=1
v4l2-ctl --set-ctrl=horizontal_flip=0
v4l2-ctl --set-ctrl=vertical_flip=0
v4l2-ctl --set-ctrl=power_line_frequency=1 #50Hz
v4l2-ctl --set-ctrl=sharpness=0 #Lowpass filter, noise reduction
v4l2-ctl --set-ctrl=backlight_compensation=0 #Hipass filter, edge enhancement
v4l2-ctl --set-ctrl=chroma_gain=3 #shutter mode (does not really work)
