#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import math
import pandas as pd
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray

# 오차 범위 설정 (미터 단위)
tolerance = 0.6
goal_x = 0
goal_y = 0
goal_z = 0

def load_room_orientations(csv_file):
    df = pd.read_csv(csv_file)
    room_orientations = {}
    for _, row in df.iterrows():
        room_id = str(row['room_id'])
        orientation_type = row['orientation_type']
        if room_id not in room_orientations:
            room_orientations[room_id] = {}
        room_orientations[room_id][orientation_type] = (
            row['x'], row['y'], row['z'], row['qx'], row['qy'], row['qz'], row['qw']
        )
    return room_orientations

# CSV 파일 경로 지정
room_orientations = load_room_orientations('/home/dfx/catkin_ws/src/my_robot/map/x_y_z.csv')

def move_to_room(room_id, orientation_type='front', frame="map"):
    global goal_x, goal_y, goal_z
    if room_id in room_orientations and orientation_type in room_orientations[room_id]:
        x, y, z, qx, qy, qz, qw = room_orientations[room_id][orientation_type]
        goal_x, goal_y, goal_z = x, y, z

        goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        rospy.sleep(1)
        goal_publisher.publish(goal)
        rospy.loginfo("Goal published for room {} with orientation {}: moving to position x={}, y={}, z={}".format(room_id, orientation_type, x, y, z))

        global status_subscriber
        status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, check_goal_status)
    else:
        rospy.loginfo("Room ID or orientation type not found.")

def check_goal_status(status_msg):
    global goal_x, goal_y, goal_z
    try:
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=3)
        current_x = current_pose.pose.pose.position.x
        current_y = current_pose.pose.pose.position.y
        current_z = current_pose.pose.pose.position.z
    except rospy.ROSException as e:
        rospy.loginfo("Failed to get current robot pose: {}".format(e))
        return
    except Exception as e:
        rospy.loginfo("An error occurred when trying to fetch the robot's current position: {}".format(e))
        return

    try:
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2 + (goal_z - current_z) ** 2)
        if distance <= tolerance:
            rospy.loginfo("Goal reached within tolerance. Shutting down.")
            rospy.signal_shutdown("Goal reached within tolerance")
        elif status_msg.status_list:
            status = status_msg.status_list[-1].status
            if status == 3:
                rospy.loginfo("Goal reached exactly.")
                rospy.signal_shutdown("Goal reached")
            elif status in [4, 5]:
                rospy.loginfo("Goal not reached but stopped trying.")
                rospy.signal_shutdown("Stopping attempts to reach goal")
    except Exception as e:
        rospy.loginfo("An error occurred during status check: {}".format(e))

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_room_node', anonymous=True)
        room_id = input("Enter the room ID (e.g., '301', '403'): ").strip()
        orientation_type = input("Enter the orientation (e.g., 'front', 'side', 'back'): ").strip()
        move_to_room(room_id, orientation_type)

        while not rospy.is_shutdown():
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt exception")
    except Exception as e:
        rospy.loginfo("An error occurred: {}".format(e))
        raise