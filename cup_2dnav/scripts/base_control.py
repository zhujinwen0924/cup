#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function, absolute_import, division
from cup import Controller
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time

human_intervation_stop = False  # 人工干预急停
current_twist = Twist()


def human_intervation_handler(data):
    # 接受人工干预停止信号
    global human_intervation_stop
    if data.data == 'STOP':
        rospy.logwarn('人工干预急停!!!')
        human_intervation_stop = True


def cb_base_control_forward(command):
    global controller, human_intervation_stop
    if human_intervation_stop:
        # 人工停止
        speed, steering = 0, 0
    else:
        # 正常情况

        # 目标速度和转角
        steering = command.angular.z * 1.0
        speed = command.linear.x * 1.0

    controller.move(speed, steering)
    rospy.loginfo('速度: {} 转角:{}'.format(speed, steering))


def cb_update_current_twist(twist):
    global current_twist
    current_twist = twist.twist


if __name__ == '__main__':
    time.sleep(2)
    try:
        rospy.init_node('nav2d_base_control_forward')
        rospy.loginfo('INIT nav2d_base_control_forward...')
        controller = Controller()
        rospy.Subscriber('/cmd_vel', Twist, callback=cb_base_control_forward, queue_size=1)
        rospy.Subscriber('/twist', TwistStamped, callback=cb_update_current_twist, queue_size=1)
        rospy.Subscriber('/BOTTLE_HUMAN_INTERVATION', String, human_intervation_handler)
        rospy.spin()
    except:
        print('KeyboardInterrupt,exit now')
        speed, steering = 0, 0
        for i in range(10):
            controller.move(speed, steering)
        exit()
