#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    pub = rospy.Publisher('/twist_mux/cmd_vel', Twist, queue_size=1)
    t = Twist()
    t.angular.z = 2 
    t.linear.x = 0.5
    for l in data.ranges:
        if l < 2:
            pub.publish(t)

def robot_avoider():
    rospy.init_node('robot_avoider', anonymous=False)
    rospy.loginfo('robot_avoider started')
    rospy.Subscriber('/scan', LaserScan, callback)
       
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        robot_avoider()
    except rospy.ROSInterruptException:
        pass