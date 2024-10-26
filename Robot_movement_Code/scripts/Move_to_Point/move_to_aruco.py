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
def move_robot():
    # Twist 메시지를 사용하여 /cmd_vel 토픽에 속도 명령을 보냄
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    
    # 로봇을 50cm 앞으로 이동시키기 위해 x축 선속도를 설정
    twist.linear.x = 0.2  # 이동 속도 (m/s), 속도는 필요에 따라 조정 가능
    twist.angular.z = 0.0  # 회전 없음
    
    # 50cm 이동에 필요한 시간 계산
    distance_to_move = 0.5  # 50cm
    move_time = distance_to_move / twist.linear.x
    
    rospy.loginfo(f"Moving forward for {move_time} seconds")
    
    # 지정한 시간 동안 이동
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < move_time:
        pub.publish(twist)
    
    # 멈추기 위한 명령
    twist.linear.x = 0.0
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
                        rospy.loginfo("Marker ID 1 detected! Moving the robot forward by 50cm.")
                        move_robot()  # 로봇을 앞으로 50cm 이동

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