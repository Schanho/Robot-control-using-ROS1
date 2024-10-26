import pyrealsense2 as rs
import numpy as np
import cv2
# Realsense 카메라의 Depth프레임
class RealSenseCapture:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)        
        self.pipeline.start(config)
        
    def get_depth_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Depth 값을 0-255 사이로 정규화하여 Grayscale로 변환
        depth_image_nomalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_nomalized = np.uint8(depth_image_nomalized)  # 값들을 8비트로 변환

        depth_color= cv2.applyColorMap(depth_image_nomalized,cv2.COLOMAP_JET)
        return depth_color

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
    try:
        while True:
            # Depth 프레임 가져오기
            depth_frame = cap.get_depth_frame()
            if depth_frame is not None:
                cv2.imshow('Depth  with Color', depth_frame)  # Grayscale로 시각화
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
