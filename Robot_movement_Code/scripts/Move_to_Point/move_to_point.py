#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray

# 호실과 방향을 매핑하는 딕셔너리
room_orientations = {
    '403': {
        'front': (-0.089263, -0.064664, 0, 0, 0, 1),
        'side': (-0.089263, -0.064664, 0, 1, 0, 0),
        'back': (-0.089263, -0.064664, 0, 0, 1, 0)
    },
    '301': {
        'front': (3.712719, -0.935105, 0, 0, 0, 1),
        'side': (3.712719, -0.935105, 0, 1, 0, 0),
        'back': (3.712719, -0.935105, 0, 0, 1, 0)
    }
}

tolerance = 0.6  # 오차 범위 설정 (미터 단위)
goal_x = 0  # 전역 변수 초기화
goal_y = 0  # 전역 변수 초기화
goal_z = 0  # 전역 변수 초기화

def move_to_room(room_id, orientation_type='front', frame="map"):
    global goal_x, goal_y, goal_z  # 전역 변수 사용 선언
    if room_id in room_orientations and orientation_type in room_orientations[room_id]:
        x, y, z, qx, qy, qz, qw = room_orientations[room_id][orientation_type]
        goal_x, goal_y, goal_z = x, y, z  # 목표 위치 설정

        goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z  # z축 좌표 추가
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
        current_z = current_pose.pose.pose.position.z  # z축 좌표 추가
    except rospy.ROSException as e:
        rospy.loginfo("Failed to get current robot pose: {}".format(e))
        return
    except Exception as e:
        rospy.loginfo("An error occurred when trying to fetch the robot's current position: {}".format(e))
        return

    try:
        # 3차원 거리 계산
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