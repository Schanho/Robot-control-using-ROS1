# map_loader.py
import rospy
import os
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class MapLoader:
    def __init__(self):
        self.clear_costmap_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    def load_map(self, map_file):
        rospy.loginfo(f"Loading map: {map_file}")
        os.system(f"rosrun map_server map_server {map_file}")
        rospy.sleep(2)  # 맵이 로딩되는 동안 잠시 대기
        self.clear_costmap_srv()  # 맵이 로딩된 후 costmap을 초기화

    def set_initial_pose(self, x, y, yaw):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        initial_pose.pose.covariance = [0.5, 0, 0, 0, 0, 0,
                                        0, 0.5, 0, 0, 0, 0,
                                        0, 0, 0.5, 0, 0, 0,
                                        0, 0, 0, 99999, 0, 0,
                                        0, 0, 0, 0, 99999, 0,
                                        0, 0, 0, 0, 0, 99999]
        
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.sleep(1)
        pub.publish(initial_pose)

    def load_map_for_floor(self, floor):
        map_file = f"/home/dfx/catkin_ws/src/my_robot/maps/floor_{floor}_map.yaml"
        self.load_map(map_file)
        if floor == 1:
            self.set_initial_pose(x=2.0, y=3.0, yaw=0.0)
        elif floor == 2:
            self.set_initial_pose(x=1.0, y=4.0, yaw=1.57)
        elif floor == 3:
            self.set_initial_pose(x=3.0, y=5.0, yaw=3.14)
        elif floor == 4:
            self.set_initial_pose(x=2.0, y=6.0, yaw=-1.57)
        # 추가적인 층에 대한 초기 위치 설정 가능