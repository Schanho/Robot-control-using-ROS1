#!/usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def publish_point_cloud(ply_file):
    rospy.init_node('ply_publisher', anonymous=True)
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

    # PLY 파일 로드
    pcd = o3d.io.read_point_cloud(ply_file)
    rospy.loginfo(f"Loaded {len(pcd.points)} points from {ply_file}")

    # 포인트 클라우드 다운샘플링
    voxel_size = 0.09  # 이 부분을 조정하서 맵의 정밀도를 조정할수있음..//  
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    points = np.asarray(downpcd.points)
    rospy.loginfo(f"Downsampled to {len(points)} points")

    # 헤더 생성
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    # PointCloud2 메시지 필드 정의
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    # PointCloud2 메시지 생성
    cloud = pc2.create_cloud(header, fields, points)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("Success Publishing point cloud")
        pub.publish(cloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_point_cloud('/home/dfx/.ros/cloud.ply')
    except rospy.ROSInterruptException:
        pass