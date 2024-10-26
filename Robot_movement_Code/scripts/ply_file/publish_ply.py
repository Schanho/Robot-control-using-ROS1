#!/usr/bin/env python

# import rospy
# import pcl
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header

# def publish_point_cloud(ply_file):
#     rospy.init_node('ply_publisher', anonymous=True)
#     pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

#     # PLY 파일 로드
#     pcl_cloud = pcl.load(ply_file)
#     points = []

#     for point in pcl_cloud:
#         points.append([point[0], point[1], point[2]])

#     # 헤더 생성
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = 'map'

#     # PointCloud2 메시지 필드 정의
#     fields = [
#         PointField('x', 0, PointField.FLOAT32, 1),
#         PointField('y', 4, PointField.FLOAT32, 1),
#         PointField('z', 8, PointField.FLOAT32, 1)
#     ]

#     # PointCloud2 메시지 생성
#     cloud = pc2.create_cloud(header, fields, points)
#     rate = rospy.Rate(1) # 1 Hz

#     while not rospy.is_shutdown():
#         pub.publish(cloud)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_point_cloud('/home/dfx/.ros/cloud.ply')
#     except rospy.ROSInterruptException:
#         pass

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
    points = np.asarray(pcd.points)

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
        pub.publish(cloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_point_cloud('/home/dfx/.ros/cloud.ply')
    except rospy.ROSInterruptException:
        pass