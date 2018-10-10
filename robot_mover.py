#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('robot_mover')

pub = rospy.Publisher('/twist_mux/cmd_vel', Twist, queue_size=1)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    t = Twist()
    t.linear.x = 2.0
    pub.publish(t)
    r.sleep()
    
