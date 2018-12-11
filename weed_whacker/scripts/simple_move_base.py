# -*- coding: utf-8 -*-
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
 
class Go_To_Point():
    def __init__(self):
       
        self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction) #start actionclient
        self.move_client.wait_for_server(rospy.Duration(5)) #wait for server to start up
   
    def point(self, P):
       
        goal = MoveBaseGoal() # new MoveBaseGoal message
        goal_pose = Pose() # new Pose message
       
        goal_pose.position.x = P[0] # fill up pose pos x
        goal_pose.position.y = P[1] # fill up pose pos y
        goal_pose.position.z = 0 # fill up pose pos z with 0
        goal_pose.orientation.w = 1 # fill up pose ori w with 1
        goal_pose.orientation.x = P[2] # fill up pose ori x with 0
        goal_pose.orientation.y = P[3] # fill up pose ori y with 0
        goal_pose.orientation.z = P[4] # fill up pose ori z with 0
       
        goal.target_pose.header.stamp = rospy.Time.now() # set MoveBaseGoal time stamp to current time from rospy
        goal.target_pose.header.frame_id = 'map' # set the frame to map
        goal.target_pose.pose = goal_pose # set the pose to the above
       
        self.move_client.send_goal(goal) # send new goal to actionlib server
       
        success = self.move_client.wait_for_result(rospy.Duration(30)) # wait up to 30s for result
       
        state = self.move_client.get_state() # get the state of the current goal
        result = False # create result bool and set it to false
       
        if success and state == GoalStatus.SUCCEEDED: # if success is true and state == succeeded
         result = True # set result as true
        else:
         self.move_client.cancel_goal() # if failure, cancle goal
       
        return result # return result
