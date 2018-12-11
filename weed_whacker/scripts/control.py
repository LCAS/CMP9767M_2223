#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class weed_control():
	
    def __init__(self):

	self.spray_pub = rospy.Publisher('/spray', Bool, queue_size=10)
	self.find_weeds_pub = rospy.Publisher('/find_weeds', Bool, queue_size=10)
	self.spray sub = rospy.Subscriber('/spray', Bool, self.spray_weeds_call)
	self.spray = False
	self.find_weeds = False

    def spray_weeds_call(self, data):
	if data.data == False:
		self.spray = False #spraying finished


rospy.init_node('weed_control')
wc = weed_control()
rospy.spin()

