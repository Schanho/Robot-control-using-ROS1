import pyrealsense2 as rs
import numpy as np
import cv2
# Realsense 카메라의 데이터 스트임을 설정하고, RGB 프레임과 깊이 정보를 가져오려고 만듦
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
    # fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
    # video = cv2.VideoWriter('cap.mp4', fourcc, 10.0, (640, 480))
    try:
        while True:
            rgb_frame = cap.get_rgb_frame()
            if rgb_frame is not None:
                cv2.imshow('RGB', rgb_frame)
                # video.write(rgb_frame)
            
            distance = cap.get_depth_distance(320, 240)
            if distance:
                print(f"Distance at (320, 240): {distance} meters")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
