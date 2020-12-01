#!/usr/bin/env python
from __future__ import print_function
import Jetson.GPIO as GPIO
import rospy
import time
import sys
from std_msgs.msg import Bool


def GPIO_init(channel=40):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(channel, GPIO.OUT)


def turn_water(state=0):
    channel = 40
    if state == 0:
        GPIO.output(channel, GPIO.LOW)
    elif state == 1:
        GPIO.output(channel, GPIO.HIGH)
    else:
        print('please input correct state!')


def callback(flag):
    if flag.data:
        turn_water(1)
    else:
        turn_water(0)


if __name__ == '__main__':
    GPIO_init()

    rospy.init_node('TX2_gpio')

    sub = rospy.Subscriber('/GPIO_STAT', Bool, callback=callback)

    rospy.spin()
