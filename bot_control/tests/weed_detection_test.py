#!/usr/bin/env python

import unittest
import rospy
from geometry_msgs.msg import PointStamped

# Test to check if kinect_processor is detecting weeds
class TestWeedDetection(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!

    def weed_location_callback(self, msg):
        self.weed_received = True
                    

    def setUp(self):
        rospy.init_node("weed_location_tester")
        self.sub = rospy.Subscriber('/thorvald_001/weed_location', PointStamped, self.weed_location_callback)
        self.weed_received = None

    def test_weed_detection(self):
        
        rate = rospy.Rate(1) # check every second
        max_seconds = 30
        count = 0
        # try fox max 30 seconds
        while not rospy.is_shutdown() and count < max_seconds:
            if self.weed_received:
                self.sub.unregister()
                self.assertTrue(True)
                return
            count += 1
            rate.sleep()
        if rate >= max_seconds:
            self.assertTrue(False, msg="timed out after %d seconds" % max_seconds)

if __name__ == '__main__':
    PKG = 'bot_control'
    import rostest
    rostest.rosrun(PKG, 'TestWeedDetection', TestWeedDetection)
