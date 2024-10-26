import numpy as np
import cv2
import matplotlib.pyplot as plt

# 파일 경로 설정
file_path = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/kalman_paticle.png'
save_path_base = '/home/dfx/catkin_ws/src/my_robot/scripts/Gmapping/map/map_base'

# 이미지 불러오기
def load_image(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    return img

# 검정색 선 엣지 강화 함수 (엣지 검출 후 강화)
def enhance_black_edges(image, kernel_size=3, iterations=2):
    # 이진화 처리 (검정색 선을 강조하기 위해 임계값 적용)
    _, binary_image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY_INV)
    
    # 엣지 검출 적용
    edges = cv2.Canny(binary_image, 50, 150)
    
    # 모폴로지 팽창 적용 (검정색 선 강화)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    enhanced_edges = cv2.dilate(edges, kernel, iterations=iterations)
    
    # 원래 이미지에 엣지 합성 (검정색 선 강화)
    enhanced_image = cv2.bitwise_or(binary_image, enhanced_edges)
    enhanced_image = cv2.bitwise_not(enhanced_image)
    
    return enhanced_image

# 튀어나온 부분 수정 함수
def remove_outliers(image, area_threshold=100):
    # 이진화 처리
    _, binary_image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY_INV)
    
    # 외곽 컨투어 찾기
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 작은 영역의 컨투어 제거
    for contour in contours:
        if cv2.contourArea(contour) < area_threshold:
            cv2.drawContours(binary_image, [contour], -1, 0, thickness=cv2.FILLED)
    
    # 원래 이미지에 반영
    cleaned_image = cv2.bitwise_not(binary_image)
    return cleaned_image

# 이미지 불러오기
original_image = load_image(file_path)

# 검정색 선 엣지 강화 적용
enhanced_image = enhance_black_edges(original_image, kernel_size=3, iterations=2)

# 튀어나온 부분 수정 적용
cleaned_image = remove_outliers(enhanced_image, area_threshold=200)

# 원본 이미지와 수정된 이미지 비교
plt.figure(figsize=(18, 6))

plt.subplot(1, 3, 1)
plt.title('Original Image')
plt.imshow(original_image, cmap='gray')
plt.axis('off')

plt.subplot(1, 3, 2)
plt.title('Enhanced Black Edges Image')
plt.imshow(enhanced_image, cmap='gray')
plt.axis('off')

plt.subplot(1, 3, 3)
plt.title('Cleaned Image')
plt.imshow(cleaned_image, cmap='gray')
plt.axis('off')

# 이미지 저장
cv2.imwrite(f"{save_path_base}_original.png", original_image)
cv2.imwrite(f"{save_path_base}_enhanced_black_edges2.png", enhanced_image)
cv2.imwrite(f"{save_path_base}_cleaned.png", cleaned_image)

plt.tight_layout()
plt.show()