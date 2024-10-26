#!/usr/bin/env python

import rospy
import matplotlib
matplotlib.use('Agg')
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from threading import Thread, Lock
import time
import math
from tf.transformations import euler_from_quaternion
from filterpy.kalman import KalmanFilter
import numpy as np


class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.state = None

    def apply(self, value):
        if self.state is None:
            self.state = value
        else:
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state

class IMUPlotter:
    def __init__(self):
        self.lock = Lock()
        rospy.init_node('imu_plotter_node', anonymous=True)
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 15), sharex=True)
        self.fig.suptitle('/imu Graph')

        self.kf_x = KalmanFilter(dim_x=2, dim_z=1)
        self.kf_y = KalmanFilter(dim_x=2, dim_z=1)
        self.kf_z = KalmanFilter(dim_x=2, dim_z=1)
        
        
        self.init_kalman_filter(self.kf_x)
        self.init_kalman_filter(self.kf_y)
        self.init_kalman_filter(self.kf_z)

        self.lines = {
            "acceleration x": self.axs[0].plot([], [])[0],
            "acceleration y": self.axs[1].plot([], [])[0],
            "acceleration z": self.axs[2].plot([], [])[0],
            "yaw": self.axs[3].plot([], [], 'r')[0]
        }

        for ax, label in zip(self.axs, self.lines.keys()):
            ax.yaxis.set_label_position("right")
            ax.set_ylabel(label)
        self.axs[3].set_ylabel('Yaw (degrees)')

        self.data = {
            "acceleration x": [],
            "acceleration y": [],
            "acceleration z": [],
            "yaw": []
        }
        self.time_data = []

        rospy.Subscriber("/imu", Imu, self.callback)
        self.ros_thread = Thread(target=rospy.spin)
        self.ros_thread.start()

        self.lpf_x=LowPassFilter(alpha=0.1)
        self.lpf_y=LowPassFilter(alpha=0.1)
        self.lpf_z=LowPassFilter(alpha=0.1)

    def init_kalman_filter(self, kf):
        kf.x = np.array([[0.], [0.]])  # 초기 상태 (위치와 속도)
        kf.F = np.array([[1., 1.],
                         [0., 1.]])  # 상태 전이 행렬
        kf.H = np.array([[1., 0.]])  # 관측 행렬
        kf.P *= 1000.  # 초기 불확실성
        kf.R = 10  # 관측 노이즈
        kf.Q = np.array([[1e-6, 0.],
                         [0., 1e-6]])  # 프로세스 노이즈

    def apply_kalman_filter(self, kf, measurement):
        kf.predict()
        kf.update(measurement)
        return kf.x[0][0]  # 필터링된 위치 값 반환

    def callback(self, msg):
        with self.lock:
            # 칼만 필터 적용
            acc_x_filtered = self.apply_kalman_filter(self.kf_x, msg.linear_acceleration.x)
            acc_y_filtered = self.apply_kalman_filter(self.kf_y, msg.linear_acceleration.y)
            acc_z_filtered = self.apply_kalman_filter(self.kf_z, msg.linear_acceleration.z)

            acc_x_lpf = self.lpf_x.apply(acc_x_filtered)
            acc_y_lpf = self.lpf_y.apply(acc_y_filtered)
            acc_z_lpf = self.lpf_z.apply(acc_z_filtered)

            self.data["acceleration x"].append(acc_x_filtered)
            self.data["acceleration y"].append(acc_y_filtered)
            self.data["acceleration z"].append(acc_z_filtered)

            euler = euler_from_quaternion([msg.orientation.x,
                                           msg.orientation.y,
                                           msg.orientation.z,
                                           msg.orientation.w])
            yaw_degrees = math.degrees(euler[2])
            self.data["yaw"].append(yaw_degrees)

            self.time_data.append(time.time())

    def update_plot(self):
        with self.lock:
            min_length = min(len(self.time_data), *[len(self.data[key]) for key in self.data])

            for idx, (key, line) in enumerate(self.lines.items()):
                line.set_data(self.time_data[:min_length], self.data[key][:min_length])
                self.axs[idx].relim()
                self.axs[idx].autoscale_view()
                if key == "acceleration z":
                    self.axs[idx].set_ylim(10,25)
                else:
                    self.axs[idx].set_ylim(-1,1)

            self.axs[-1].set_xlabel("Time (s)")

        plt.draw()
        plt.pause(0.01)
        plt.savefig('/home/dfx/catkin_ws/src/my_robot/map/imu_graph_kalman.png')

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.update_plot()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    plotter = IMUPlotter()
    plotter.run()