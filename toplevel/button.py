#!/usr/bin/env python3

# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO
import time
import subprocess,signal
from subprocess import PIPE
# import rospy
# from std_msgs.msg import Bool
# import os


# Pin definitions
button = 29

# Globals
p = None
running = 0


# def click():
#     if GPIO.input(button) == 0:
#         print("click")
#     else:
#         print("wait for click")


def both(channel):
    global running
    global p
    if GPIO.input(button)==0:
        if running == 0:
            running = 1
            # Check if any Tmux process is running, if not start launch script else don't do anything
            if subprocess.run(["tmux","has-session","-t","record"], stdout=PIPE,stderr=PIPE).returncode != 0:
                print("Starting process")
                p = subprocess.Popen(["/home/airlab/ws/src/toplevel/scripts/start_scripts.sh"])
    else:
        if p:
            print("Killing process")
            p = subprocess.Popen(["/home/airlab/ws/src/toplevel/scripts/stop_scripts.sh"])
            running = 0
            





def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    # GPIO.setup([led_pin_1], GPIO.OUT)  # LED pins set as output
    GPIO.setup(button, GPIO.IN)  # button pin set as input

    # Initial state for LEDs:
    # GPIO.output(led_pin_1, GPIO.LOW)
    GPIO.add_event_detect(button, GPIO.BOTH, callback=both, bouncetime=200)
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            pass
            # if GPIO.input(button)==0:
            #     if running:
            #         print("logging")
            #     else:
            #         print("Logging on but not running")
            # else:
            #     print("not logging")
            # time.sleep(0.1)

    finally:
        print("Ending button script...")
        GPIO.cleanup()  # cleanup all GPIOs

if __name__ == '__main__':
    main()
