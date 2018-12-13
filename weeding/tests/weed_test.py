#!/usr/bin/env/python

import unittest
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import sqrt

class WeedTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("weed_tester")
        self.sub = rospy.Subscriber('/spray', Bool, self.spray_callback )
	self.pub = rospy.Publisher('/spray', Bool, queue_size=10)
        self.spray = False

    def spray_callback(self, msg):
	if msg.data == True and self.spray == False:
		self.spray = True	

    def test_spray(self):
	self.pub.publish(False)
	rate = rospy.Rate(1)
	max_seconds = 240
	count = 0
	while not rospy.is_shutdown() and count < max_seconds:
		if self.spray == True:
			self.assertTrue(True)
			self.sub.unregister()
			return
		count = += 1
		rate.sleep()
	if rate >= max_seconds:
		self.assertTrue(False, msg="timed out after %d seconds" % max_seconds)

if __name__ == '__main__':
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'WeedTest', WeedTest)
