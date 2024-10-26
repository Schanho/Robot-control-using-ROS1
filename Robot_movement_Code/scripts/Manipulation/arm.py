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

    while not arm.joint_states.position:
        rospy.loginfo("Waiting for joint_states...")
        rate.sleep()

    try:
        while not rospy.is_shutdown(): 
            rospy.loginfo("-------------------- Manipulator Start --------------------")
            current_joint_angles = [math.radians(angle) for angle in arm.joint_states.position]
            rospy.loginfo("Current joint angles:")
            print(current_joint_angles)
            
            cmd = input("Enter 'x', 'z', 'c' to start moving, 'w' to move to previous position, or 'q' to quit: ")

            if cmd == "x": 
                print(f"Input: {cmd}")
                joint_positions = [2.0, -0.52, -0.05, 0.12, -0.71, 2.0]
                arm.move_joint(joint_positions)
                print(f"joint 1 : {joint_positions[1]}", f"joint 2 : {joint_positions[2]}", f"joint 3 : {joint_positions[3]}", f"joint 4 : {joint_positions[4]}")
                rate.sleep()

            elif cmd == "z": 
                print(f"Input: {cmd}")
                joint_positions = [1.0, -0.12, 0.90, -0.52, -0.71, 2.0]
                arm.move_joint(joint_positions)
                print(f"joint 1 : {joint_positions[1]}", f"joint 2 : {joint_positions[2]}", f"joint 3 : {joint_positions[3]}", f"joint 4 : {joint_positions[4]}")
                rate.sleep()

            elif cmd == "c": 
                print(f"Input: {cmd}")
                joint_positions = [2.0, 0.12, -0.57, 1.20, 0.16, .0]
                arm.move_joint(joint_positions)
                print(f"joint 1 : {joint_positions[1]}", f"joint 2 : {joint_positions[2]}", f"joint 3 : {joint_positions[3]}", f"joint 4 : {joint_positions[4]}")
                rate.sleep()
            elif cmd =="a":
                print(f"Input: {cmd}")
                joint_positions=[]
            # inital pose
            elif cmd == "w":
                print(f"Input: {cmd}")
                joint_positions = [2.0, 0.0, -1.57, 1.20, 0.6, 2.0]
                arm.move_joint(joint_positions)
                print(f"joint 1 : {joint_positions[1]}", f"joint 2 : {joint_positions[2]}", f"joint 3 : {joint_positions[3]}", f"joint 4 : {joint_positions[4]}")
                rate.sleep()

            elif cmd == "q":  # quit command
                rospy.loginfo("-------------------- Manipulator Quit --------------------")
                rospy.signal_shutdown('exit')
                sys.exit()

            else:
                rospy.loginfo("Invalid command. Try again.")

    except KeyboardInterrupt:
        rospy.loginfo("-------------------- Manipulator Quit --------------------")

if __name__ == "__main__":
    main()