#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf_conversions

if __name__ == '__main__':
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('tf2_goal_publisher')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # try:
        #     trans = tfBuffer.lookup_transform('map', 'ball_absolute', rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rate.sleep()
        #     continue

        location = PoseStamped()

        location.header.seq = 1
        location.header.stamp = rospy.Time.now()
        location.header.frame_id = "map"

        location.pose.position.x = float(0.1)
        location.pose.position.y = float(0)
        location.pose.position.z = float(0)
        location.pose.orientation.x = float(0)
        location.pose.orientation.y = float(0)
        location.pose.orientation.z = float(0)
        location.pose.orientation.w = float(1)

        pub.publish(location)

        rate.sleep()