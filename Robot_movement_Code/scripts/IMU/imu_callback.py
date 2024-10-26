#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from collections import deque

class IMU_Average:
    def __init__(self):
        self.sub = rospy.Subscriber('/imu', Imu, self.callback)
        self.window_size = 10  # 이 부분이 내가 받은 imu 값을 평균화할 데이터 갯수 부분 
        self.x_vals = deque(maxlen=self.window_size)
        self.y_vals = deque(maxlen=self.window_size)
        self.z_vals = deque(maxlen=self.window_size)
        self.rate = rospy.Rate(1)

    def callback(self, data):
        self.x_vals.append(data.linear_acceleration.x)
        self.y_vals.append(data.linear_acceleration.y)
        self.z_vals.append(data.linear_acceleration.z)
        

        avg_x = sum(self.x_vals) / len(self.x_vals)
        avg_y = sum(self.y_vals) / len(self.y_vals)
        avg_z = sum(self.z_vals) / len(self.z_vals)

        rospy.loginfo(f"Average X: {avg_x}, Y: {avg_y}, Z: {avg_z}")

if __name__ == '__main__':
    rospy.init_node('imu_average_node')
    IMU_Average()
    rospy.spin()