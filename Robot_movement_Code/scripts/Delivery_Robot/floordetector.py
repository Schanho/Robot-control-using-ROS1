# floor_detector.py
import rospy
from sensor_msgs.msg import Imu
import math

class FloorDetector:
    def __init__(self):
        self.current_floor = None
        rospy.Subscriber("/imu", Imu, self.callback)

    def determine_floor(self, z_value):
        if 10.3 <= z_value <= 10.7:
            return 1
        elif 20.3 <= z_value <= 20.7:
            return 2
        elif 30.3 <= z_value <= 30.7:
            return 3
        elif 40.3 <= z_value <= 40.7:
            return 4
        # 추가 층에 대한 조건 추가 가능
        else:
            return None

    def callback(self, msg):
        z_value = msg.linear_acceleration.z
        detected_floor = self.determine_floor(z_value)
        if detected_floor is not None and detected_floor != self.current_floor:
            self.current_floor = detected_floor
            rospy.loginfo(f"Detected floor: {detected_floor}")
        return self.current_floor

    def update_floor(self):
        return self.current_floor