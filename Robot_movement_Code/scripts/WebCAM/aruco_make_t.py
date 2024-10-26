import cv2
import cv2.aruco as aruco

# ArUco 마커 딕셔너리 설정 (4x4 사이즈, 50개의 마커)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 이미지 저장 경로 설정
save_path = '/home/dfx/catkin_ws/src/my_robot/scripts/WebCAM/makers'

# 0번부터 3번까지의 마커 이미지 생성
for i in range(4):
    # generateImageMarker를 사용하여 마커 생성
    img = aruco.generateImageMarker(aruco_dict, i, 200)
    cv2.imwrite(f'{save_path}marker_{i}.png', img)