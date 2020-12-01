#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function, division, absolute_import

import math
import sys

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import pickle
import yaml

# 路点列表
from scripts.waypoint_editor import visualize_waypoint

waypoints = None

task_table = None

goal_idx = 1

# 是否绕圈
CIRCLE = True


def cb_waypoint_publish(result):
    global waypoints, task_table, goal_idx
    if result.status.status == 3:

        # 到达目标之后停止
        print('Achieved goal: {}'.format(goal_idx - 1))
        rospy.loginfo('goal reached, executing task, sleep [{}]s...'.format(task_table[goal_idx - 1]['time']))

        # 发布已经完成的goal idx
        achieved_goal_idx = Int8()
        achieved_goal_idx.data = goal_idx - 1
        pub_task.publish(achieved_goal_idx)

        # 停止等待任务
        rospy.sleep(task_table[goal_idx - 1]['time'])
        rospy.loginfo('task done, move to next goal...')

        # 并清除costmap

        if goal_idx < len(waypoints):
            pub_goal.publish(waypoints[goal_idx])
            goal_idx += 1
        else:
            rospy.logwarn('已完成所有waypoint的发布\n')
            # 如果绕圈则返回第一个路点
            if CIRCLE:
                pub_goal.publish(waypoints[0])
                goal_idx = 1
                rospy.logwarn('循环模式 返回waypoint 0\n')

        rospy.loginfo('已完成waypoint: {} / {}'.format(goal_idx - 1, len(waypoints)))
        rospy.loginfo('下一个目标: ({},{})\n\n'
                      .format(waypoints[goal_idx - 1].pose.position.x, waypoints[goal_idx - 1].pose.position.y))
        visualize_waypoint(waypoints, pub_poses_text, pub_poses, )


def cb_click_initial_point(point):
    global waypoints, goal_idx
    # 找离click point最近的一个路点
    min_dist = 1e8
    min_dist_index = 0
    for idx, pose in enumerate(waypoints):
        dist = math.sqrt(
            (point.point.x - pose.pose.position.x) ** 2 +
            (point.point.y - pose.pose.position.y) ** 2
        )
        if dist < min_dist:
            min_dist = dist
            min_dist_index = idx

    # 从该路点开始导航
    pub_goal.publish(waypoints[min_dist_index])
    goal_idx = min_dist_index + 1
    rospy.loginfo('选择了 #{} waypoint'.format(min_dist_index))


def load_map(path):
    pose_array = []
    with open(path) as map_file:
        # pose_array_msg = yaml.load(map_file.read(), Loader=yaml.FullLoader)
        pose_array_msg = yaml.load(map_file.read())
        # 解析yaml到pose ayyay
        # pose_array.header.frame_id = pose_array_msg['header']['frame_id']
        for pose_msg in pose_array_msg['poses']:
            # 解析pose
            pose = PoseStamped()
            pose.header.frame_id = pose_array_msg['header']['frame_id']
            pose.pose.position.x = pose_msg['position']['x']
            pose.pose.position.y = pose_msg['position']['y']
            pose.pose.orientation.z = pose_msg['orientation']['z']
            pose.pose.orientation.w = pose_msg['orientation']['w']

            pose_array.append(pose)

        # 解析task table
        task = pose_array_msg['task']
        print(task)
    return pose_array, task


if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')

    # 订阅到达指定目标事件
    sub_reach_goal_event = rospy.Subscriber('/move_base/result', MoveBaseActionResult, cb_waypoint_publish)
    sub_point = rospy.Subscriber('/clicked_point', PointStamped, cb_click_initial_point)

    # 发布路点
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # 可视化publisher
    pub_poses_text = rospy.Publisher("/waypoints/text", MarkerArray, queue_size=10)
    pub_poses = rospy.Publisher("/waypoints", PoseArray, queue_size=10)
    pub_task = rospy.Publisher("/achieved_area_idx", Int8, queue_size=1)

    # 载入waypoint
    if len(sys.argv) > 1:
        waypoints, task_table = load_map('/home/yellow/Code/bottle_ws/src/cup_2dnav/maps/' + sys.argv[1] + '.yaml')
        rospy.loginfo('载入地图. -- 长度{}'.format(len(waypoints)))
    else:
        rospy.logerr('请指定地图文件路径!!!!!!')
        raise IOError()
    rospy.sleep(0.5)
    # 可视化全部路点
    visualize_waypoint(waypoints, pub_poses_text, pub_poses, )

    rospy.loginfo('使用RVIZ的Publish Point选择一个点, 将从距离选择点最近的waypoint开始导航\n\n')
    rospy.spin()

