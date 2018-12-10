#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class landmark_detector:
    odom_data = None

    def __init__(self):    

        self.thorvald_vel = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', 
            Twist, queue_size=1)
        rospy.Subscriber('/thorvald_001/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/thorvald_001/odometry/gazebo', Odometry, self.odom_callback)

    def scan_callback(self, data):
        if not self.odom_data:
            return

        angle = np.argmin(data.ranges)
        distance = data.ranges[angle]
        angle = np.deg2rad(angle - 360)

        del_x = distance * np.sin(np.deg2rad(self.odom_data.pose.pose.orientation.z * 360) + angle)
        del_y = distance * np.cos(np.deg2rad(self.odom_data.pose.pose.orientation.z * 360) + angle)

        # vel_data = Twist()
        print("X: ", del_x, "      Y: ", del_y) 
        # self.odom_data.pose.pose.orientation.z

        # self.thorvald_vel.publish(vel_data)
    
    def odom_callback(self, data):
        self.odom_data = data


def main(args):
    '''Landmark Detector node'''
    rospy.init_node('landmark_detector', anonymous=True)
    kp = landmark_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)