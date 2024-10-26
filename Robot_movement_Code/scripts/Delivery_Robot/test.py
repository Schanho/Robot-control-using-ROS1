#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

class LaserFilterNode:
    def __init__(self):
        # 초기화
        rospy.init_node('laser_filter_node', anonymous=True)
        
        # 필터링된 데이터를 퍼블리시할 토픽 생성
        self.pub_filtered_scan = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        self.pub_pointcloud = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=10)
        
        # 원본 데이터를 구독
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Laser to PointCloud 변환기
        self.laser_projector = LaserProjection()

    def scan_callback(self, scan_data):
        # 후처리: 필터링 과정 (각도 및 거리 필터)
        filtered_scan = self.apply_filters(scan_data)
        
        # 필터링된 데이터를 퍼블리시
        self.pub_filtered_scan.publish(filtered_scan)

        # PointCloud로 변환하여 퍼블리시 (Rviz에서 PointCloud로 시각화)
        pointcloud = self.laser_projector.projectLaser(filtered_scan)
        self.pub_pointcloud.publish(pointcloud)

    def apply_filters(self, scan_data):
        # 각도와 거리 필터링
        new_scan = LaserScan()
        new_scan.header = scan_data.header
        new_scan.angle_min = scan_data.angle_min
        new_scan.angle_max = scan_data.angle_max
        new_scan.angle_increment = scan_data.angle_increment
        new_scan.time_increment = scan_data.time_increment
        new_scan.scan_time = scan_data.scan_time
        new_scan.range_min = scan_data.range_min
        new_scan.range_max = scan_data.range_max

        # 필터링된 데이터 저장
        new_ranges = []
        for r in scan_data.ranges:
            # 필터링 조건: 예를 들어 0.2m ~ 5m 범위만 통과
            if r >= 0.2 and r <= 5.0:
                new_ranges.append(r)
            else:
                new_ranges.append(float('inf'))  # 필터된 값을 inf로 설정

        new_scan.ranges = new_ranges
        new_scan.intensities = scan_data.intensities
        
        return new_scan

if __name__ == '__main__':
    try:
        laser_filter_node = LaserFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass