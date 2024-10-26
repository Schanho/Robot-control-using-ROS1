#!/usr/bin/env python
import open3d as o3d
import numpy as np
import cv2

# PLY 파일 로드
pcd = o3d.io.read_point_cloud("/home/dfx/.ros/cloud.ply")
print(f"Loaded {len(pcd.points)} points from /home/dfx/.ros/cloud.ply")

# 포인트 클라우드를 numpy 배열로 변환
points = np.asarray(pcd.points)
# Z 좌표를 제거하여 2D 평면으로 투영
points_2d = points[:, :2]       

# 포인트 클라우드의 최대 및 최소 값을 사용하여 맵의 크기 결정
min_x, min_y = np.min(points_2d, axis=0)
max_x, max_y = np.max(points_2d, axis=0)

# 맵의 해상도 설정 (예: 0.01m)
resolution = 0.01

# 맵의 크기 계산
width = int((max_x - min_x) / resolution)
height = int((max_y - min_y) / resolution)

# Occupancy Grid 맵 초기화
occupancy_grid = np.zeros((height, width), dtype=np.uint8)

# 포인트를 Occupancy Grid 맵에 반영
for point in points_2d:
    x, y = point
    grid_x = int((x - min_x) / resolution)
    grid_y = int((y - min_y) / resolution)
    # 경계 조건 추가
    if 0 <= grid_x < width and 0 <= grid_y < height:
        occupancy_grid[grid_y, grid_x] = 255  # 점이 있는 위치를 255로 설정

# 맵을 이미지로 저장
occupancy_grid = cv2.flip(occupancy_grid, 1) # 좌우반전
occupancy_grid = cv2.rotate(occupancy_grid, cv2.ROTATE_180) #오른쪽으로 180 돌람
cv2.imwrite("/home/dfx/.ros/occupancy_gr2id.pgm", occupancy_grid)


# YAML 파일 생성
with open("/home/dfx/.ros/map.yaml", 'w') as yaml_file:
    yaml_file.write("image: /home/dfx/.ros/occupancy_grid.pgm\n")
    yaml_file.write("resolution: 0.01\n")
    yaml_file.write("origin: [0.0, 0.0, 0.0]\n")
    yaml_file.write("negate: 0\n")
    yaml_file.write("occupied_thresh: 0.65\n")
    yaml_file.write("free_thresh: 0.196\n")