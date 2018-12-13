#!/usr/bin/env/python

import unittest
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import sqrt

class WeedTest(unittest.TestCase):
    def test_two_two_two(self):
	self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'WeedTest', WeedTest)
