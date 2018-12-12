#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#ACTIVATED 3RD
class endofrow:

    def __init__(self):
        #initialise subscribers
        #color_sub to use amount of plant seen to scale row
        #move_sub to detect when the process has finished to initialize next step
        self.color_sub = rospy.Subscriber("/colorval_camera", Float32, self.callback)
        #self.move_sub = rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, self.move_callback)
        
    def callback(self, data):
        #set up movement publisher
        pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        t = Twist()
        #show reference data
        print(data.data)
        #if plant seen stop moving forward and start moving up the row
        if data.data > 0.3:
            t.linear.x = 0.0
            t.linear.y = 0.4
            pub.publish(t)
            #if too close too many plants will be seen and it will turn away
            if data.data > 7:
                t.linear.y = 0.0
                t.angular.z = 0.6
                pub.publish(t)
            else:
                #if too far away turn towards
                if data.data < 1:
                    t.linear.y = 0.0
                    t.angular.z = -0.6
                    pub.publish(t)
                else:
                    #if an ok value drive sideways with no turning
                    t.angular.z = 0.0
                    pub.publish(t)
                    
    def move_callback(self, data):
        #publisher to next stage
        pub = rospy.Publisher("/stage_indicator", Int8, queue_size=1)
        if data.twist.twist.linear.x < 0.03:
            if data.twist.twist.linear.y < 0.03:
                if data.twist.twist.angular.z < 0.03:
                    #make new stage when finished then exit
                    pub.publish(2)
                    rospy.signal_shutdown("End of Row Found!")
                    
rospy.init_node('endofrow', anonymous=True, disable_signals=True)
eor = endofrow()
rospy.spin()
                    