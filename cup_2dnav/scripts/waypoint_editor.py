#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function, division

import json
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker

WIDTH, HEIGHT = 4320, 4000
ORIGIN = np.array([-100, -100])
RESOLUTION = 0.05
pose_array = PoseArray()
pose_array.header.frame_id = 'map'
pose_persist = []

# 保存各个目标点的元信息
task_table = []

# 地图文件存储路径
MAP_SAVING_PATH = "/tmp/map.yaml"


def visualize_waypoint(waypoints, pub_poses_text, pub_poses, ):
    marker_array = MarkerArray()
    pose_array = PoseArray()
    pose_array.header.frame_id = '/map'
    for idx, pose in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration()
        marker.ns = "pose_marker"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Marker的尺寸，单位是m
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        # Marker的位置姿态
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y + 0.8
        marker.pose.position.z = 0

        marker.text = '#{}'.format(idx)

        # Marker的颜色和透明度
        marker.color.r = 1
        marker.color.a = 1

        marker_array.markers.append(marker)
        pose_array.poses.append(pose.pose)

    pub_poses_text.publish(marker_array)
    pub_poses.publish(pose_array)
    # while not rospy.is_shutdown():
    #     rospy.sleep(0.1)


def cb_waypoint_publish(pose):
    global pose_array, task_table
    pose_array.poses.append(pose.pose)
    rospy.loginfo(pose_array)

    # 元信息
    task_table.append([len(pose_persist), 2.0, 'area_name'])

    # 保存路径点
    pose_persist.append(pose)
    visualize_waypoint(pose_persist, pub_poses_text, pub_poses, )

    with open(MAP_SAVING_PATH, 'w') as map_yaml:
        map_content = 'task:\n'
        for item in task_table:
            map_content += '- {{ id: {}, time: {}, name: {} }}\n'.format(item[0], item[1], item[2])
        map_content += pose_array.__str__()

        map_yaml.write(map_content)


if __name__ == '__main__':
    rospy.init_node('waypoint')

    print(sys.argv)
    # 读取地图文件的存储路径
    if len(sys.argv) > 1:
        MAP_SAVING_PATH = sys.argv[1]
        rospy.loginfo('地图文件: {}'.format(MAP_SAVING_PATH))
    else:
        rospy.logerr('地图文件路径错误!')
        raise IOError

    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, cb_waypoint_publish)

    pub_poses_text = rospy.Publisher("/waypoints/text", MarkerArray, queue_size=10)
    pub_poses = rospy.Publisher("/waypoints", PoseArray, queue_size=10)

    rospy.spin()
