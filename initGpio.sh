# unload modules (reset)
modprobe -r spidev
modprobe -r spi-s3c64xx
# load modules for spi
modprobe spi-s3c64xx
modprobe spidev

# export IRQ gpio
echo 199 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio199/direction
echo rising > /sys/class/gpio/gpio199/edge

