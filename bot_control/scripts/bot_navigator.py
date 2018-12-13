#!/usr/bin/env python

# Python libs
import sys, math

# OpenCV
import cv2

# Ros libraries
import rospy, actionlib, tf

# Ros Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Twist
from std_srvs.srv import Empty

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class BotNavigation:
    odom = None
    odom_map = None
    forward_vel = Twist()
    weed_list = [] # local list of unique weeds
    row_start_pose = None
    bot_state = None
    possible_states = {
        "Finding" : 0,
        "Aligning" : 1,
        "Weeding" : 2,
        "Switching" : 3
    }

    def __init__(self):
        self.bot_state = self.possible_states["Weeding"]
        self.forward_vel.linear.x = 0.3
        self.forward_vel.linear.y = 0
        self.forward_vel.linear.z = 0
        self.forward_vel.angular.x = 0
        self.forward_vel.angular.y = 0
        self.forward_vel.angular.z = 0

        self.match_tolerance_radius = 0.0025 # square of max distance between two points to be considered the same weed

        # transform listener
        self.tf_listener = tf.TransformListener()

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.sprayer = rospy.ServiceProxy('/thorvald_001/spray', Empty)

        # Publisher for thorvald_001/twist_mux/cmd_vel to move thorvald with velocities
        self.pub_twist_mux = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', 
            Twist, queue_size=1)
        
        rospy.Subscriber("/thorvald_001/odometry/gazebo",
            Odometry, self.odom_callback)
        
        # Subscriber for weed points detected in kinect_processor.py
        rospy.Subscriber("/thorvald_001/weed_location",
            PointStamped, self.weed_loc_callback)

    # removes first weed from weed_list
    def weed_passed(self):
        if len(self.weed_list) > 0:     # check if weed_list is not empty to prevent exception in pop(i)
            self.weed_list.pop(0)
    
    # callback for /thorvald_001/weed_location subscriber
    # looks for weed in weed_list. Append to it if not found.
    def weed_loc_callback(self, data):
        if self.bot_state != self.possible_states["Weeding"]:
            return
        for i, weed in enumerate(self.weed_list):
            if ((weed.point.x - data.point.x)**2 + (weed.point.x - data.point.x)**2) < self.match_tolerance_radius:
                self.weed_list[i].point.x = (weed.point.x + data.point.x)/2
                self.weed_list[i].point.y = (weed.point.y + data.point.y)/2
                return
        
        weed = data
        self.weed_list.append(weed)
        # self.weed_list.insert(sort_index, weed)
        # if sort_index == 0:
        #     self.move_client.stop_tracking_goal()
        # print(len(self.weed_list))

    def navigate_to_weeds(self):
        if len(self.weed_list) == 0:
            self.keep_moving()
            return
        try:
            temp_point = self.weed_list[0]
            temp_point.header.stamp = rospy.Time(0)
            base_link_weed = self.tf_listener.transformPoint('thorvald_001/base_link', temp_point)
            if base_link_weed.point.x < -0.75:
                self.weed_list.pop(0)
                return
        except tf.ExtrapolationException as e:
            print(e)
            return
        
        dist = self.sq_dist_to_point(self.weed_list[0], self.odom_map)
        # print(dist)
        if dist < 0.75:
            # print(self.weed_list[0], len(self.weed_list))
            state, result = self.move_bot(self.weed_list[0].point.x, self.weed_list[0].point.y, self.row_start_pose.pose.orientation.z)
            if result:
                self.sprayer()
                self.weed_passed()
                print("Weed Sprayed")
                return
            else:
                # print("Goal not reached", state)
                return
        else:
            self.keep_moving()

    # returns the square of distance to point. Saving computation by skipping sqrt
    def sq_dist_to_point(self, pnt, pse):
        dx = pnt.point.x - pse.pose.position.x
        dy = pnt.point.y - pse.pose.position.y
        return dx**2 + dy**2

    # returns the square of distance between poses. Saving computation by skipping sqrt
    def sq_dist_bw_poses(self, pse1, pse2):
        dx = pse1.pose.position.x - pse2.pose.position.x
        dy = pse1.pose.position.y - pse2.pose.position.y
        return dx**2 + dy**2

    def odom_callback(self, data):
        odom = PoseStamped()
        odom.header = data.header
        odom.header.stamp = rospy.Time(0)
        odom.pose = data.pose.pose
        try:
            self.odom_map = self.tf_listener.transformPose('map', odom)
            self.odom = self.tf_listener.transformPose("thorvald_001/base_link", odom)
        except tf.ExtrapolationException as e:
            print(e)
            return
        except tf.LookupException as e:
            print(e)
            return
        if not self.row_start_pose:
            self.row_start_pose = self.odom_map


    def move_bot(self, x, y, theta):
        goal = MoveBaseGoal()
        goal_pose = Pose()
       
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.position.z = 0
        goal_pose.orientation.w = 1
        goal_pose.orientation.x = 0
        goal_pose.orientation.y = 0
        goal_pose.orientation.z = theta
       
        goal.target_pose.header.stamp = rospy.Time.now() # set MoveBaseGoal time stamp to current time from rospy
        goal.target_pose.header.frame_id = 'map' # set the frame to map
        goal.target_pose.pose = goal_pose # set the pose to the above

        # send new goal to actionlib server
        # execute, preempt timeout set to 10s
        self.move_client.send_goal(goal)

        # wait up to 90s for result
        success = self.move_client.wait_for_result(rospy.Duration(15))
        
        state = self.move_client.get_state() # get the state of the current goal
        result = False # create result bool and set it to false
        
        if success and state == GoalStatus.SUCCEEDED: # if success is true and state == succeeded
            result = True # set result as true
        else:
            self.move_client.cancel_goal() # cancel goal
       
        return state, result # return result
        
    def keep_moving(self):
        if self.sq_dist_bw_poses(self.row_start_pose, self.odom_map) > 8.0:
            self.bot_state = self.possible_states["Switching"]
        self.pub_twist_mux.publish(self.forward_vel)

def main(args):
    '''Row Navigation Node'''
    rospy.init_node('RowNav', anonymous=True)
    bot_nav = BotNavigation()
    try:
        r = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            if bot_nav.odom and bot_nav.odom_map:
                # print("nav")
                if bot_nav.bot_state == bot_nav.possible_states["Weeding"]:
                    bot_nav.navigate_to_weeds()
                
            r.sleep()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)