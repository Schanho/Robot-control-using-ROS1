#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
#euler_from_quaternion 함수 추가: 이 함수는 쿼터니언을 롤, 피치, 요 각도로 변환하는 데 사용됨. 

def pose_callback(data):
    # Extract position
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    # Extract orientation
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # Extract eoa and sda
    eoa = data.pose.covariance[0]  # covariance 배열의 첫 번째 값을 eoa로 사용하도록 수정해봄
    sda = data.pose.covariance[7]  # covariance 배열의 여덟 번째 값을 sda로 사용하도록 수정해봄

    rospy.loginfo("Robot Current Pose: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, eoa: %f, sda: %f]", x, y, z, roll, pitch, yaw, eoa, sda)
    
    

def listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
