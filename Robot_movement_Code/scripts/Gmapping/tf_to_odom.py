#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

rospy.init_node('tf_to_odom')
listener = tf.TransformListener()
odom_pub = rospy.Publisher('/tf_to_odom', Odometry, queue_size=10)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = trans[0]
        odom.pose.pose.position.y = trans[1]
        odom.pose.pose.position.z = trans[2]
        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]
        odom_pub.publish(odom)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(f"TF Lookup Exception: {e}")
    rate.sleep()
