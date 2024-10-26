from PIL import Image
import numpy as np
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt

# 파일 경로 설정
file_path_ground_truth = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/filter/map12.pgm'
file_path_basic_slam = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/filter/map.pgm'

# 이미지 불러오기 함수
def load_image_as_array(image_path, target_size=None):
    img = Image.open(image_path)
    img = img.convert('L')  # 흑백 이미지로 변환
    if target_size:
        img = img.resize(target_size, Image.LANCZOS)  # 이미지 크기 조정
  # 이미지 크기 조정
    return np.array(img)

# 데이터 불러오기 (ground_truth의 크기를 기준으로 다른 이미지를 맞춤)
ground_truth = load_image_as_array(file_path_ground_truth)
target_size = ground_truth.shape[::-1]  # (width, height)
basic_slam = load_image_as_array(file_path_basic_slam, target_size)

# 경로 오차 지표 계산 함수
def calculate_metrics(ground_truth, estimated):
    # ATE (Absolute Trajectory Error)
    ate = np.mean(np.abs(ground_truth - estimated))

    # RMSE (Root Mean Squared Error)
    rmse = np.sqrt(mean_squared_error(ground_truth.flatten(), estimated.flatten()))

    return ate, rmse

# 각각의 SLAM에 대한 ATE와 RMSE 계산
metrics_basic = calculate_metrics(ground_truth, basic_slam)

# 결과 출력
print("Basic SLAM - ATE: {:.4f}, RMSE: {:.4f}".format(metrics_basic[0], metrics_basic[1]))

# 시각화
labels = ['Basic SLAM']
ates = [metrics_basic[0]]
rmses = [metrics_basic[1]]

x = np.arange(len(labels))
width = 0.35

fig, ax = plt.subplots(figsize=(10, 6))
rects1 = ax.bar(x - width/2, ates, width, label='ATE', color='blue')
rects2 = ax.bar(x + width/2, rmses, width, label='RMSE', color='red')

# 그래프 레이블 설정
ax.set_xlabel('SLAM Methods')
ax.set_ylabel('Error')
ax.set_title('Comparison of ATE and RMSE for Different SLAM Methods')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

# 그래프 표시
plt.grid(True)
plt.show()
