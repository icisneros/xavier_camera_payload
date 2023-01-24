# Xavier NX Custom Carrier GPIO Script
This ROS node  exposes the GPIO pins and publishes a message on button press.

The GPIO mapping is as follows with respect to the [Xavier NX 40-pin header](https://www.jetsonhacks.com/nvidia-jetson-xavier-nx-gpio-header-pinout/):

- 32, GPIO424: Blue LED
- 29, GPIO421: Button
- 11, GPIO428: Pin 5 (RTS) on UART 1. Mapped to topic /gpio/gpio1. Controls LED Driver for lighting on ORDV2.
- 36, GPIO429: Pin 4 (CTS) on UART 1. Mapped to topic /gpio/gpio2. 


# Setting up GPIO478 UART0_RTS
This isn't available in the rPi driver so we have to use a system node access approach for this. Ensure your platform has the following path available, or run the following commands to add it:

- /sys/class/gpio/gpio478/

If not there run:

- sudo echo 478 > /sys/class/gpio/export

gpio-button.py will attempt the above without sudo and should be successful, this may not be necessary.
Because UART0_RTS is already an output, no pinmux changes are required.
