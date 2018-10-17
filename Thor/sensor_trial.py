#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
    pub = rospy.Publisher("twist_mux/cmd_vel", Twist, queue_size=1)
    t = Twist()
    t.linear.x = 2.0
    
    if data.ranges[360] < 4:
        t.angular.z = 2.0
        pub.publish(t)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
    
    
listener()