#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
import os

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

def main():
    cap = RealSenseCapture()

    # ArUco 마커 딕셔너리 선택 (4x4 사이즈, 50개의 마커)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # 동영상 파일 저장 경로 설정
    save_path = '/home/dfx/catkin_ws/src/my_robot/scripts/Manipulation/output.avi'
    os.makedirs(os.path.dirname(save_path), exist_ok=True)  # 디렉토리 없을 경우 생성
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 코덱 설정 (XVID는 .avi 확장자를 사용)
    out = cv2.VideoWriter(save_path, fourcc, 30.0, (640, 480))  # 파일명, FPS, 프레임 크기 설정

    try:
        while True:
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

                # 인식된 마커 ID 출력 및 조건별 동작 수행 (1번에서 4번 ID만 인식)
                for i, marker_id in enumerate(ids):
                    marker_id_value = marker_id[0]  # marker_id는 리스트이므로 값 추출

                    if 1 <= marker_id_value <= 4:
                        print(f"Marker ID {marker_id_value} detected! Performing action for ID {marker_id_value}.")
                    else:
                        print(f"Marker ID {marker_id_value} ignored (not in range 1-4).")

            # 결과 영상 출력
            resized_frame = cv2.resize(rgb_frame, (0, 0), fx=2.0, fy=2.0)  # 이미지 크기를 두 배로 확대
            cv2.imshow('RGB Frame with ArUco Markers', resized_frame)

            # 동영상 프레임 저장 (resized_frame이 아닌 원래 크기 rgb_frame으로 저장)
            out.write(rgb_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.stop()
        out.release()  # 동영상 파일을 저장하고 닫음
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
