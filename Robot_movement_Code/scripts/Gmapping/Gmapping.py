#!/usr/bin/env python

import rospy
import os
from subprocess import Popen

def start_gmapping():
    # ROS 노드 초기화
    rospy.init_node('gmapping_slam_python', anonymous=True)
    
    # Gmapping SLAM 시작을 위해 필요한 파라미터 설정
    model = rospy.get_param("~model", "waffle")
    base_frame = rospy.get_param("~base_frame", "base_footprint")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    map_frame = rospy.get_param("~map_frame", "map")

    # gmapping 노드 실행
    gmapping_cmd = [
        "rosrun", "gmapping", "slam_gmapping",
        "_base_frame:={}".format(base_frame),
        "_odom_frame:={}".format(odom_frame),
        "_map_frame:={}".format(map_frame),
    ]
    
    # 서브프로세스를 이용해 gmapping 실행
    Popen(gmapping_cmd)

    rospy.loginfo("Gmapping SLAM started using Python script")
    
    # ROS 스핀을 통해 노드가 종료되지 않도록 유지
    rospy.spin()

if __name__ == "__main__":
    try:
        start_gmapping()
    except rospy.ROSInterruptException:
        pass
