#!/usr/bin/env python

import unittest
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt

# A sample python unit test
class TestNav(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!

    def odom_callback(self, msg):
        #rospy.loginfo('received new odometry: %s' % msg)
        if not self.start_pose:
            self.start_pose = msg.pose.pose
        self.current_pose = msg.pose.pose
                    

    def setUp(self):
        rospy.init_node("nav_tester")
        self.sub = rospy.Subscriber('/thorvald_001/odometry/gazebo', Odometry, self.odom_callback )
        self.pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=10)
        self.start_pose = None


    def distance(self):
        if not self.start_pose:
            return 0.0
        p1 = self.start_pose
        p2 = self.current_pose
        diff = [
            (p1.position.x - p2.position.x) ** 2,
            (p1.position.y - p2.position.y) ** 2
        ]
        return sqrt(diff[0]+diff[1])

    def test_one_meter_nav(self):
        
        rate = rospy.Rate(1) # check every second
        max_seconds = 10
        count = 0
        twist = Twist()
        twist.linear.x = 1
        # try fox max 30 seconds
        while not rospy.is_shutdown() and count < max_seconds:
            self.pub.publish(twist)
            d = self.distance()
            rospy.loginfo('distance travelled: %f' % d)
            if d > 1.0:
                self.sub.unregister()
                self.assertTrue(True)
                return
            count += 1
            rate.sleep()
        if rate >= max_seconds:
            self.assertTrue(False, msg="timed out after %d seconds" % max_seconds)

if __name__ == '__main__':
    PKG = 'uol_cmp9767m_base'
    import rostest
    rostest.rosrun(PKG, 'TestNav', TestNav)
