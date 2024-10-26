import pandas as pd
from sklearn.metrics import mean_squared_error
import numpy as np
import matplotlib.pyplot as plt

# 파일 경로 설정
file_path_ground_truth = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/ground_truth_and_estimated_path.csv'
file_path_basic_slam = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/slam_basic.csv'
file_path_kalman_slam = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/slam_kalman.csv'
file_path_improved_slam = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/slam_improved.csv'

# 데이터 불러오기
df_ground_truth = pd.read_csv(file_path_ground_truth)
df_basic_slam = pd.read_csv(file_path_basic_slam)
df_kalman_slam = pd.read_csv(file_path_kalman_slam)
df_improved_slam = pd.read_csv(file_path_improved_slam)

# 경로 오차 지표 계산
def calculate_metrics(ground_truth_x, ground_truth_y, estimated_x, estimated_y):
    # ATE (Absolute Trajectory Error)
    ate = np.mean(np.sqrt((ground_truth_x - estimated_x) ** 2 + (ground_truth_y - estimated_y) ** 2))
    
    # RMSE (Root Mean Squared Error)
    rmse = np.sqrt(mean_squared_error(ground_truth_x, estimated_x) + mean_squared_error(ground_truth_y, estimated_y))
    
    return ate, rmse

# 각각의 SLAM에 대한 ATE와 RMSE 계산
metrics_basic = calculate_metrics(df_ground_truth['Ground_Truth_X'], df_ground_truth['Ground_Truth_Y'], 
                                  df_basic_slam['Estimated_X'], df_basic_slam['Estimated_Y'])

metrics_kalman = calculate_metrics(df_ground_truth['Ground_Truth_X'], df_ground_truth['Ground_Truth_Y'], 
                                   df_kalman_slam['Estimated_X'], df_kalman_slam['Estimated_Y'])

metrics_improved = calculate_metrics(df_ground_truth['Ground_Truth_X'], df_ground_truth['Ground_Truth_Y'], 
                                     df_improved_slam['Estimated_X'], df_improved_slam['Estimated_Y'])

# 시각화
labels = ['Basic SLAM', 'Kalman Filter SLAM', 'Improved SLAM']
ates = [metrics_basic[0], metrics_kalman[0], metrics_improved[0]]
rmses = [metrics_basic[1], metrics_kalman[1], metrics_improved[1]]

x = np.arange(len(labels))
width = 0.4

# ATE 비교 그래프
fig, ax_ate = plt.subplots(figsize=(10, 6))
rects_ate = ax_ate.bar(x, ates, width, color='blue')

# 그래프 레이블 설정
ax_ate.set_xlabel('SLAM Methods')
ax_ate.set_ylabel('ATE (m)')
ax_ate.set_title('Comparison of ATE for Different SLAM Methods')
ax_ate.set_xticks(x)
ax_ate.set_xticklabels(labels)

# ATE 그래프 저장
plt.grid(True)
plt.savefig('/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/ate_comparison.png')
plt.close()

# RMSE 비교 그래프
fig, ax_rmse = plt.subplots(figsize=(10, 6))
rects_rmse = ax_rmse.bar(x, rmses, width, color='red')

# 그래프 레이블 설정
ax_rmse.set_xlabel('SLAM Methods')
ax_rmse.set_ylabel('RMSE (m)')
ax_rmse.set_title('Comparison of RMSE for Different SLAM Methods')
ax_rmse.set_xticks(x)
ax_rmse.set_xticklabels(labels)

# RMSE 그래프 저장
plt.grid(True)
plt.savefig('/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/rmse_comparison.png')
plt.close()