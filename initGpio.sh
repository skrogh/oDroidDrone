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
[ -d "/sys/class/gpio/gpio199" ]&& echo "already exported" || echo 199 > /sys/class/gpio/export
# set as input
echo in > /sys/class/gpio/gpio199/direction
# have to be superuser for this one
echo rising > /sys/class/gpio/gpio199/edge
