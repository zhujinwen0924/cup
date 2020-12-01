#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
import time

import sys

v = 0.5

frame_id = 0


def callback(twist):
    global v, init_time
    global frame_id
    linear_x = twist.twist.linear.x
    flag = Bool()
    if linear_x < v:
        flag.data = False
        pub.publish(flag)
        frame_id = 0
    else:
        frame_id += 1
        if int(frame_id / 10) % 2 == 0:
            flag.data = True
            pub.publish(flag)
        else:
            flag.data = False
            pub.publish(flag)


if __name__ == '__main__':
    rospy.init_node('GPIO_Control')
    print('init')

    sub = rospy.Subscriber("twist", TwistStamped, callback=callback)
    pub = rospy.Publisher("/GPIO_STAT", Bool, queue_size=10)

    rospy.spin()
