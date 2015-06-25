#!/bin/bash
# must be root
# run with: su -c "sh initGpio.sh"

# unload modules (reset)
modprobe -r spidev
modprobe -r spi-s3c64xx
# load modules for spi
modprobe spi-s3c64xx
modprobe spidev

# export IRQ gpio, only if not yet exported
if [ -d "/sys/class/gpio/gpio199" ]
then
 echo "already exported"
else
 echo "creating ramdisk"
 mount -t tmpfs -o size=512m tmpfs /mnt/ramdisk
 ln -s /mnt/ramdisk/logs ~/oDroidDrone/eigenOpenCv
 mkdir /mnt/ramdisk/logs
 #export
 echo "exporting gpios"
 echo 199 > /sys/class/gpio/export
 # set as input
 echo in > /sys/class/gpio/gpio199/direction
 # have to be superuser for this one
 echo rising > /sys/class/gpio/gpio199/edge
fi
