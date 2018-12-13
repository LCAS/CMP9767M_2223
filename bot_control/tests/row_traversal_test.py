#!/usr/bin/env python

import unittest
import rospy
from nav_msgs.msg import Odometry
from math import sqrt

# Tests the combined work of kinect_processor and bot_navigator to 
# check if one whole row is traversed (length 8m) from (-4.0,2.0) to () 
# and there is no significant deviation in the bot's position from the row
class TestRowTraversal(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!

    def odom_callback(self, msg):
        #rospy.loginfo('received new odometry: %s' % msg)
        if not self.start_pose:
            self.start_pose = msg.pose.pose
        self.current_pose = msg.pose.pose
                    

    def setUp(self):
        rospy.init_node("row_traversal_test")
        self.sub = rospy.Subscriber('/thorvald_001/odometry/gazebo', Odometry, self.odom_callback )
        self.start_pose = None


    def deviation(self):
        if not self.start_pose:
            return False
        
        diff = self.start_pose.position.y - self.current_pose.position.y
        return diff > 0.7 or diff < -0.7

    def distance_from_end(self):
        if not self.start_pose:
            return False
        p1 = self.start_pose
        p2 = self.current_pose
        diff = [
            # goal is 8 m from starting point
            (p1.position.x + 8.0 - p2.position.x) ** 2,
            (p1.position.y - p2.position.y) ** 2
        ]
        return sqrt(diff[0]+diff[1])

    def test_row_nav(self):
        
        rate = rospy.Rate(1) # check every second
        max_seconds = 300
        count = 0
        # try fox max 300 seconds
        while not rospy.is_shutdown() and count < max_seconds:
            d = self.deviation()
            if d:
                self.sub.unregister()
                self.assertTrue(False, msg="Robot deviated from row")
                return
            if self.distance_from_end() < 0.4:
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
    rostest.rosrun(PKG, 'TestRowTraversal', TestRowTraversal)
