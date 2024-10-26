#!/usr/bin/env python

import rospy
import matplotlib
matplotlib.use('Agg')
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from threading import Thread, Lock
import time

class OdomPlotter:

    def __init__(self):
        self.lock = Lock()
        rospy.init_node('odom_plotter_node', anonymous=True)
        self.fig, self.axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
        self.fig.suptitle('/odom Graph')

        self.lines = {
            "position x": self.axs[0].plot([], [])[0],
            "position y": self.axs[1].plot([], [])[0],
            "position z": self.axs[2].plot([], [])[0],
            "orientation x": self.axs[3].plot([], [])[0],
            "orientation y": self.axs[4].plot([], [])[0],
            "orientation z": self.axs[5].plot([], [])[0],
            "orientation w": self.axs[6].plot([], [])[0]
        }

        for ax, label in zip(self.axs, self.lines.keys()):
            ax.yaxis.set_label_position("right")
            ax.set_ylabel(label)

        self.data = {
            "position x": [],
            "position y": [],
            "position z": [],
            "orientation x": [],
            "orientation y": [],
            "orientation z": [],
            "orientation w": []
        }
        self.time_data = []

        rospy.Subscriber("/odom", Odometry, self.callback)
        self.ros_thread = Thread(target=rospy.spin)
        self.ros_thread.start()
#position 값: 로봇의 현재 위치를 나타내는 값
#orientation 값: 로봇의 회전(자세)을 나타내며, 쿼터니언 형식으로 제공 
    def callback(self, msg):
        with self.lock:
            self.data["position x"].append(msg.pose.pose.position.x)
            self.data["position y"].append(msg.pose.pose.position.y)
            self.data["position z"].append(msg.pose.pose.position.z)
            self.data["orientation x"].append(msg.pose.pose.orientation.x)
            self.data["orientation y"].append(msg.pose.pose.orientation.y)
            self.data["orientation z"].append(msg.pose.pose.orientation.z)
            self.data["orientation w"].append(msg.pose.pose.orientation.w)
            self.time_data.append(time.time())

    def update_plot(self):
        with self.lock:
            for idx, (key, line) in enumerate(self.lines.items()):
                line.set_data(self.time_data, self.data[key])
                self.axs[idx].relim()
                self.axs[idx].autoscale_view()
            self.axs[-1].set_xlabel("Time (s)")

        plt.draw()
        plt.pause(0.01)
        plt.savefig('/home/dfx/catkin_ws/src/my_robot/map/graph.png')

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.update_plot()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    plotter = OdomPlotter()
    plotter.run()