#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(data):
    rospy.loginfo("Robot Current Pose: [%f, %f, %f]", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.w)

def listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()