#!/usr/bin/env python
import open3d as o3d
import octomap
import numpy as np
import os

# PLY 파일 경로
ply_file = "/home/dfx/.ros/cloud.ply"

# PLY 파일 로드
print("Loading PLY file...")
pcd = o3d.io.read_point_cloud(ply_file)
print("PLY file loaded.")

points = np.asarray(pcd.points, dtype=np.float64)
print(f"Number of points: {len(points)}")

# Octomap 생성 (해상도 설정: 예를 들어 0.1 미터)
tree = octomap.OcTree(0.1)
print("Octomap created.")

# Point Cloud 데이터를 Octomap에 삽입
for point in points:
    tree.updateNode((point[0], point[1], point[2]), True)
print("Point Cloud data inserted into Octomap.")

# Occupancy 업데이트
tree.updateInnerOccupancy()
print("Inner occupancy updated.")

# Octomap 저장 경로 설정
save_directory = "/home/dfx/catkin_ws/src/my_robot/map"
if not os.path.exists(save_directory):
    os.makedirs(save_directory)  # 디렉토리가 없는 경우 생성
print(f"Saving Octomap to {save_directory}...")

octomap_file = os.path.join(save_directory, "octomap.bt").encode('utf-8')  # 경로를 바이트로 변환
tree.writeBinary(octomap_file)
print("Octomap saved.")