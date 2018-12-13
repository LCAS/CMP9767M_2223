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
    def setUp(self):
        rospy.init_node("weed_tester")
        self.sub = rospy.Subscriber('/spray', Bool, self.spray_callback )
	self.pub = rospy.Publisher('/spray', Bool, queue_size=10)
	self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction) #start actionclient
        self.move_client.wait_for_server(rospy.Duration(5)) #wait for server to start up
        self.spray = False

    def spray_callback(self, msg):
	if msg.data == True and self.spray == False:
		self.spray = True	

    def test_move_base(self):
	goal = MoveBaseGoal() # new MoveBaseGoal message
	goal_pose = Pose() # new Pose message
       
        goal_pose.position.x = 1 # fill up pose pos x
        goal_pose.position.y = 0 # fill up pose pos y
        goal_pose.position.z = 0 # fill up pose pos z with 0
        goal_pose.orientation.w = 1 # fill up pose ori w with 1
        goal_pose.orientation.x = 0 # fill up pose ori x with 0
        goal_pose.orientation.y = 0 # fill up pose ori y with 0
        goal_pose.orientation.z = 0 # fill up pose ori z with 0
       
        goal.target_pose.header.stamp = rospy.Time.now() # set MoveBaseGoal time stamp to current time from rospy
        goal.target_pose.header.frame_id = 'map' # set the frame to map
        goal.target_pose.pose = goal_pose # set the pose to the above

	self.move_client.send_goal(goal) # send new goal to actionlib server
       
        success = self.move_client.wait_for_result(rospy.Duration(60)) # wait up to 30s for result
       
        state = self.move_client.get_state() # get the state of the current goal

        if success and state == GoalStatus.SUCCEEDED: # if success is true and state == succeeded
        	self.assertTrue(True)
        else:
		self.assertTrue(False, msg="timed out after %d seconds" % max_seconds)

    def test_spray(self):#test to see if line find
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
