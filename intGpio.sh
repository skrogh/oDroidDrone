# unload modules (reset)
modprobe -r spidev
modprobe -r spi-s3c64x
# load modules for spi
modprobe spi-s3c64xx
modprobe spidev

# export IRQ gpio
echo 199 > /sys/class/gpio/export


