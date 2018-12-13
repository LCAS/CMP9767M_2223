#!/usr/bin/env python

import unittest
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# A sample python unit test
class TestWeeds(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!
    def setUp(self):
	rospy.init_node("simple_test")
	self.spray = False#
	self.sub = rospy.Subscriber('/spray', Bool, self.spray_callback)#
	self.pub = rospy.Publisher('/spray', Bool, queue_size=10)#
	self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction) #start actionclient
        self.move_client.wait_for_server(rospy.Duration(5)) #wait for server to start up

    def spray_callback(self, data):#
	self.spray = data.data#

    def test_spraying(self):#
        rate = rospy.Rate(1)#
	max_seconds = 10#
	count = 0#
	self.pub.publish(False)#
	while not rospy.is_shutdown() and count < max_seconds:#
		if self.spray == True:#
			self.sub.unregister()#
			self.assertTrue(True)#
			return#
		count += 1#
		rate.sleep()#
	self.assertTrue(False, msg="timeout")#

    def test_y_y_h(self):
	self.assertTrue(True)

    def test_move_base(self):#test move_base
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

if __name__ == '__main__':
    #PKG = 'test_roslaunch'
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestWeeds)
