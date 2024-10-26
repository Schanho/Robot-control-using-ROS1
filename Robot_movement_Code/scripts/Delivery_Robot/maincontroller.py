# main_controller.py

import rospy
from IMU.imu_kalmanfilter import IMUPlotter  # IMUPlotter 클래스를 가져옴
from Delivery_Robot.floordetector import FloorDetector  # FloorDetector 클래스를 가져옴

class MainController:
    def __init__(self):
        self.imu_plotter = IMUPlotter()
        self.floor_detector = FloorDetector(self.imu_plotter)

    def run(self):
        rospy.init_node('main_controller_node')
        rate = rospy.Rate(1)  # 1Hz로 루프 실행
        while not rospy.is_shutdown():
            current_floor = self.floor_detector.update_floor()
            if current_floor is not None:
                rospy.loginfo(f"Currently on floor: {current_floor}")
            rate.sleep()

if __name__ == '__main__':
    controller = MainController()
    controller.run()