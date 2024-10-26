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
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 깊이 스트림 설정
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # RGB 스트림 설정
        self.pipeline.start(config)

        # Align을 위한 기능 추가 (Depth to Color)
        self.align = rs.align(rs.stream.color)

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)  # RGB와 깊이 프레임 정합
        return aligned_frames

    def get_rgb_frame(self):
        aligned_frames = self.get_aligned_frames()
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def get_depth_distance(self, x, y):
        aligned_frames = self.get_aligned_frames()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            return None
        # (x, y) 좌표에서 깊이값 반환
        return depth_frame.get_distance(x, y)

    def stop(self):
        self.pipeline.stop()

# 로봇 제어 클래스
class RobotControl:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def rotate(self, angle_degree):
        velocity_message = Twist()
        angular_speed = 0.5  # 회전 속도 (라디안/초)
        relative_angle = np.radians(angle_degree)  # 목표 회전 각도
        velocity_message.angular.z = angular_speed if angle_degree > 0 else -angular_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < relative_angle:
            self.velocity_publisher.publish(velocity_message)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            self.rate.sleep()

        velocity_message.angular.z = 0  # 회전 멈춤
        self.velocity_publisher.publish(velocity_message)

    def move_forward(self, distance):
        velocity_message = Twist()
        linear_speed = 0.2  # 이동 속도 (m/s)
        velocity_message.linear.x = linear_speed

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while current_distance < distance:
            self.velocity_publisher.publish(velocity_message)
            t1 = rospy.Time.now().to_sec()
            current_distance = linear_speed * (t1 - t0)
            self.rate.sleep()

        velocity_message.linear.x = 0  # 이동 멈춤
        self.velocity_publisher.publish(velocity_message)

def main():
    cap = RealSenseCapture()
    robot = RobotControl()

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
                    # 코너의 좌표를 사용해 중앙 좌표 계산
                    corner_points = corners[i][0]  # 각 마커의 네 코너 좌표 (4x2 array)
                    center_x = int(np.mean(corner_points[:, 0]))  # x 좌표의 평균
                    center_y = int(np.mean(corner_points[:, 1]))  # y 좌표의 평균

                    # 마커의 중심에 빨간색 점 찍기
                    cv2.circle(rgb_frame, (center_x, center_y), 5, (0, 0, 255), -1)

                    # 깊이 값 반환 및 출력
                    depth_distance = cap.get_depth_distance(center_x, center_y)

                    # 깊이 값이 0.4미터 이하인 경우, 로봇이 왼쪽으로 90도 회전, 앞으로 0.8미터 이동 후, 오른쪽으로 90도 회전, 앞으로 0.7미터 이동
                    if depth_distance is not None and depth_distance<0.4:
                        print(f"Marker ID {marker_id[0]} detected at {depth_distance:.2f} meters")
                        robot.rotate(90)
                        robot.move_forward(0.80)
                        robot.rotate(-90)
                        robot.move_forward(0.70)
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