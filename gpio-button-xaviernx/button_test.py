#!/usr/bin/env python2

import RPi.GPIO as GPIO
import time
import subprocess,signal
from subprocess import PIPE
# import rospy
# from std_msgs.msg import Bool
import os



button = 29




def click():
    if GPIO.input(button) == 0:
        print("click")
    else:
        print("wait for click")


if __name__ == '__main__':

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(button, GPIO.IN)


    while 1:
        click()
        t = 1000000
        while t:
            t -= 1