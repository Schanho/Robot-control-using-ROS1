#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
import time
import math
import sys
import numpy as np

class ArmController:
    def __init__(self):
        self.joint_trajectory_pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=10)
        self.joint_move_time_pub = rospy.Publisher('/joint_move_time', Float64, queue_size=10)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.joint_states = JointState()

    def joint_states_callback(self, data):
        self.joint_states = data

    def move_joint(self, joint_positions):
        joint_pos_msg = Float64MultiArray()
        joint_pos_msg.data = joint_positions[:-1]
        self.joint_trajectory_pub.publish(joint_pos_msg)

        move_time_msg = Float64()
        move_time_msg.data = joint_positions[-1]
        print(move_time_msg.data)
        self.joint_move_time_pub.publish(move_time_msg)

def main():
    rospy.init_node('Manipulator')
    arm = ArmController()
    rate = rospy.Rate(10)

    # 대기: joint_states가 올 때까지 대기
    while not arm.joint_states.position:
        rospy.loginfo("Waiting for joint_states...")
        rate.sleep()

    # 기본 joint 상태 값 저장
    current_joint_positions = list(arm.joint_states.position)

    try:
        while not rospy.is_shutdown(): 
            rospy.loginfo("-------------------- Manipulator Start --------------------")
            rospy.loginfo("Current joint angles:")
            print(current_joint_positions)
            cmd = input("Enter 'w', 'a', 's', 'd', 'x' to control the arm, or any other key to exit: ")

            # 위로 2cm 이동 (Z축 증가)
            if cmd == "w": 
                rospy.loginfo("Moving up by 2cm")
                current_joint_positions[3] += 0.02  # Z축 이동
                arm.move_joint(current_joint_positions + [5.0])  # 2.0은 움직이는 시간
                rate.sleep()

            # 왼쪽으로 20도 회전 (yaw 회전)
            elif cmd == "a": 
                rospy.loginfo("Rotating left by 20 degrees")
                current_joint_positions[0] += math.radians(20)  # Yaw 축 회전
                arm.move_joint(current_joint_positions + [5.0])
                rate.sleep()

            # 정지
            elif cmd == "s":
                rospy.loginfo("Stopping")
                # 아무 동작도 하지 않음
                rate.sleep()

            # 오른쪽으로 20도 회전 (yaw 회전)
            elif cmd == "d":
                rospy.loginfo("Rotating right by 20 degrees")
                current_joint_positions[0] -= math.radians(20)  # Yaw 축 회전
                arm.move_joint(current_joint_positions + [5.0])
                rate.sleep()

            # 아래로 2cm 이동 (Z축 감소)
            elif cmd == "x":
                rospy.loginfo("Moving down by 2cm")
                current_joint_positions[3] -= 0.02  # Z축 이동
                arm.move_joint(current_joint_positions + [5.0])
                rate.sleep()

            else:
                rospy.loginfo("-------------------- Manipulator Quit --------------------")
                rospy.signal_shutdown('exit')
                sys.exit()
        
    except KeyboardInterrupt:
        rospy.loginfo("-------------------- Manipulator Quit --------------------")

if __name__ == "__main__":
    main()