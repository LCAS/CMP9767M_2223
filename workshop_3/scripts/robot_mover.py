#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



def scan_callback(data):
    # callback function for scan data
    pub = rospy.Publisher("/twist_mux/cmd_vel", Twist, queue_size = 1)
    
    t = Twist()        
    t.linear.x = 3.0    
        
    if data:
        for dt in data.ranges:
            if dt < 4:
                t.linear.x = 1.0
            if dt < 1:
                t.linear.x = 0
                t.angular.z = 2.0
                
    pub.publish(t)    
    
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.loginfo('working')
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()
    
listener()