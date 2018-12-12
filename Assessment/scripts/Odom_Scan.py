#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

    
def orientation_callback(data):
    pub = rospy.Publisher("/Orient_Laser", Int32MultiArray, queue_size=1)
    orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    pub.publish(orientation)
    
    
def laser_callback(data):
    pub = rospy.Publisher("/Orient_Laser", Int32MultiArray, queue_size=1)
    laser = [data.ranges]
    pub.publish(laser)
    
def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/thorvald_001/scan", LaserScan, laser_callback)
    rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, orientation_callback)
    rospy.spin()
    
listener()