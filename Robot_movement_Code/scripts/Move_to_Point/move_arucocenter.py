#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Twist

# Realsense 카메라의 데이터 스트림을 설정하고, RGB 프레임과 깊이 정보를 가져오는 클래스
class RealSenseCapture:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def get_rgb_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def get_depth_distance(self, x, y):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        return depth_frame.get_distance(x, y)

    def stop(self):
        self.pipeline.stop()

# 로봇 이동 명령을 보내는 함수
def move_robot(linear_speed, angular_speed, duration):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = linear_speed  # 전진 또는 후진 속도
    twist.angular.z = angular_speed  # 좌우 회전 속도

    # 이동 시간만큼 명령을 보냄
    rate = rospy.Rate(10)  # 10Hz
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(twist)
        rate.sleep()

    # 이동 후 정지
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

def main():
    rospy.init_node('aruco_marker_mover', anonymous=True)
    
    cap = RealSenseCapture()

    # ArUco 마커 딕셔너리 선택 (4x4 사이즈, 50개의 마커)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    try:
        while not rospy.is_shutdown():
            rgb_frame = cap.get_rgb_frame()
            if rgb_frame is None:
                continue

            # 그레이스케일로 변환
            gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)

            # ArUco 마커 탐지
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # 인식된 마커가 있으면
            if ids is not None:
                # 마커에 코너와 ID 그리기
                rgb_frame = aruco.drawDetectedMarkers(rgb_frame, corners, ids)

                # 인식된 마커 ID 출력 및 조건별 동작 수행
                for i, marker_id in enumerate(ids):
                    if marker_id == 1:  # ID 1 마커가 인식되면
                        rospy.loginfo("Marker ID 1 detected! 마커의 중심부로 이동해보겠슴다!")

                        # 마커 중심 계산    
                        corners = corners[i][0]
                        marker_center_x = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4
                        marker_center_y = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4

                        # 화면 중심 계산(이 부분이 잘되는지.. log 찍어보려함)
                        frame_center_x = rgb_frame.shape[1] / 2

                        # 좌우 회전: 마커가 화면 중앙에 맞춰지도록
                        if marker_center_x < frame_center_x - 10:
                            rospy.loginfo("Marker is to the left. Rotating right.")
                            move_robot(0, -0.2, 0.1)  # 좌회전
                        elif marker_center_x > frame_center_x + 10:
                            rospy.loginfo("Marker is to the right. Rotating left.")
                            move_robot(0, 0.2, 0.1)  # 우회전
                        else:
                            # 마커가 중앙에 있으면 앞으로 전진
                            distance = cap.get_depth_distance(int(marker_center_x), int(marker_center_y))
                            if distance > 0.5:
                                rospy.loginfo(f"Moving forward. Distance to marker: {distance} meters")
                                move_robot(0.2, 0, 0.1)  # 전진
                            else:
                                rospy.loginfo("Robot is close enough to the marker. Stopping.")
                                move_robot(0, 0, 0)  # 멈춤

            # 결과 영상 출력
            cv2.imshow('RGB Frame with ArUco Markers', rgb_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()