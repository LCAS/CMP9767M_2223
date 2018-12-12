#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float32, Int8

from geometry_msgs.msg import Twist

#ACTIVATED 4TH
class rowmove:

    def __init__(self):
        #initialise subs
        #color_sub to use amount of plants seen to go forward down the row
        #ind_sub to allow initialisation of steps
        self.color_sub = rospy.Subscriber("/colorval_camera", Float32, self.callback)
        self.ind_sub = rospy.Subscriber("/stage_indicator", Int8, self.ind_callback)
        
    def ind_callback(self, data):
        #movement publisher
        pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        t = Twist()
        #if prevous indicator is done
        if data.data == 2:
            t.linear.x = 0.6
            t.angular.z = -pi/4
            pub.publish(t)
    
    def callback(self, data):
        pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        t = Twist()
        #if enough of the row is seen to move forward
        if data.data > 6:
            t.linear.x = 0.2
            t.angular.z = 0
            pub.publish(t)
            #else:
             #   if #bottom of row:
              #      t.linear.x = 0.4
               #     t.angular.z = -pi/4
                #    pub.publish(t)

        
#activation
rospy.init_node('rowmove')
row = rowmove()
rospy.spin()

