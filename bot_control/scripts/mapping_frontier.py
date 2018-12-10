#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError

class mapping_frontier:
    odom_data = None

    def __init__(self):    
        self.bridge = CvBridge()
        self.thorvald_vel = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', 
            Twist, queue_size=1)
        self.map_image = rospy.Publisher('/map_image', 
            Image, queue_size=1)
        rospy.Subscriber('/thorvald_001/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/thorvald_001/odometry/gazebo', Odometry, self.odom_callback)

    def scan_callback(self, data):
        if not self.odom_data:
            return

        map = cv2.create(2000, 2000, cv2.CV_8U)
        # map = np.zeros([2000, 2000, 1])
        # map[:,:,0] = np.ones([2000,2000])*128/255.0


        # map = cv2.resize(map, (0,0), fx=1.0, fy=1.0)
        # map = cv2.fromarray(map)
        # cv2.imshow("Map",map)

        # angle = np.argmin(data.ranges)
        # distance = data.ranges[angle]
        # angle = np.deg2rad(angle - 360)

        # del_x = distance * np.sin(np.deg2rad(self.odom_data.pose.pose.orientation.z * 360) + angle)
        # del_y = distance * np.cos(np.deg2rad(self.odom_data.pose.pose.orientation.z * 360) + angle)

        # vel_data = Twist()
        # print("X: ", del_x, "      Y: ", del_y) 
        # self.odom_data.pose.pose.orientation.z

        # self.thorvald_vel.publish(vel_data)
        try:
            ros_img = self.bridge.cv2_to_imgmsg(map, "mono8")
        except CvBridgeError as e:
            print(e)
            return

        self.map_image.publish(ros_img)

    
    def odom_callback(self, data):
        self.odom_data = data


def main(args):
    '''Landmark Detector node'''
    rospy.init_node('mapping_frontier', anonymous=True)
    mf = mapping_frontier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)