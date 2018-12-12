#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Int8, String

#ACTIVATED 2ND
class mover:
    def __init__(self):
        #subscribers identified
        #color_sub for plant identification and sensor_sub for wall detection
        self.color_sub = rospy.Subscriber("/colorval_camera", Float32, self.callback2)
        self.sensor_sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.callback)

    def callback(self, data):
        #call publishers used 
        pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
        ind_pub = rospy.Publisher("/stage_indicator", Int8, queue_size=1)
        #initialise indicator
        ind_pub.publish(0)
        #preset move to cover area 'randomly'
        t = Twist()
        t.linear.x = 0.3
        t.angular.z = math.pi/64
        pub.publish(t)
        
        #specify movement if wall detected in sense of laser scanner
        val, idx = min((val, idx) for (idx, val) in enumerate(data.ranges))
        if val < 1.5:
            t.linear.x = 0.0
            t.angular.z = math.pi/4
            pub.publish(t)
            
    def callback2(self, data):
        #creating instance to exit so futue proesses arent interfered
        pub = rospy.Publisher("/mover_state", String, queue_size=1)
        if data.data > 0.3:
            pub.publish("Plant Found")
            rospy.signal_shutdown('plant found')
            


#activation
rospy.init_node('mover', anonymous=True, disable_signals=True)
m = mover()
rospy.spin()