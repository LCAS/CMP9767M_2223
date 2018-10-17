#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def robot_runner():
    rospy.init_node('robot_runner', anonymous=False)
    rospy.loginfo('robot_runner started')
    pub = rospy.Publisher('/twist_mux/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    t = Twist()
    while not rospy.is_shutdown():
        t.linear.x = 3.0
        
        pub.publish(t)
        r.sleep()
        
if __name__ == '__main__':
    try:
        robot_runner()
    except rospy.ROSInterruptException:
        pass
