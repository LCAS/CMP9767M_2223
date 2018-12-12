#!/usr/bin/env python

import rospy
import cv2
import numpy
import math
import message_filters
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(color_data, odom_data):
    #solve all perception
    pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
    t = Twist()
    if color_data.data < 0.4:
        if abs(odom_data.pose.pose.orientation.z) > 0.7:
        t.linear.x = 0.6
        t.angular.z = -pi/4
        pub.publish(t)
        else:
            if abs(odom_data.pose.pose.orientation.z) < 0.3:
                t.linear.x = 0.4
                t.angular.z = -pi/4
                pub.publish(t)
     else:
         if color_data.data > 6:
             t.linear.x = 0.2
             t.angular.z = 0
             pub.publish(t)


color_sub = message_filters.Subscriber("/colorval_camera", Float32)
odom_sub = message_filters.Subscriber("/thorvald_001/odometry/gazebo", Odometry)
        
ts = message_filters.TimeSynchronizer([color_sub, odom_sub], 10)
ts.registerCallback(callback)
rospy.spin()