#!/usr/bin/env python

# OpenCV
import cv2

# Ros libraries
import rospy, actionlib, tf

# Ros Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Twist
from bot_control.msg import WeedList, UniqueWeed
from std_srvs.srv import Empty

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

class RowNav:
    odom = None
    odom_map = None
    forward_vel = Twist()
    weed_list = WeedList()
    stop_nav = False

    def __init__(self):
        self.forward_vel.linear.x = 0.15
        self.forward_vel.linear.y = 0
        self.forward_vel.linear.z = 0
        self.forward_vel.angular.x = 0
        self.forward_vel.angular.y = 0
        self.forward_vel.angular.z = 0

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.sprayer = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        # Service to let weed_tracker know when a weed is sprayed/skipped
        self.weed_passed_srv = rospy.ServiceProxy('/bot_control/weed_passed', Empty)

        # Publisher for thorvald_001/twist_mux/cmd_vel to move thorvald with velocities
        self.pub_twist_mux = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', 
            Twist, queue_size=1)

        # Subscriber for weed list
        rospy.Subscriber("/thorvald_001/weed_list",
            WeedList, self.weed_list_callback)
        
        rospy.Subscriber("/thorvald_001/odometry/gazebo",
            Odometry, self.odom_callback)

        # transform listener
        self.tf_listener = tf.TransformListener()

    def navigation_controller(self):
        while(True):
            if self.stop_nav:
                break
            if not self.odom or not self.odom_map:
                continue
            if len(self.weed_list.weeds) == 0:
                
                if self.odom_map.pose.position.y > -0.75:
                    self.forward_vel.linear.y = -0.15
                elif self.odom_map.pose.position.y < -0.85:
                    self.forward_vel.linear.y = 0.15
                else:
                    self.forward_vel.linear.y = 0
                self.pub_twist_mux.publish(self.forward_vel)
                continue
            print("Have points")
            base_link_weed = self.tf_listener.transformPoint('thorvald_001/base_link', self.weed_list.weeds[0].point)
            base_link_weed.point.x += 0.5
            self.weed_list.weeds[0].point = self.tf_listener.transformPoint('map', base_link_weed)
            print("Transformations done")
            if self.weed_list.weeds[0].confidence > 8:
                print(self.weed_list.weeds[0].point.point.x, self.weed_list.weeds[0].point.point.y, )
                result = self.move_bot(self.weed_list.weeds[0].point.point.x, self.weed_list.weeds[0].point.point.y, 0)
                if result:
                    self.sprayer()
                    self.weed_passed_srv()
                    self.weed_list.pop(0)
                    print("Weed Sprayed")
            # elif (base_link_weed.point.x - self.odom.pose.pose.position.x) < 0.5:
            #     self.weed_passed_srv()
            #     self.weed_list.pop(0)
            #     print("Transformations done")


    def weed_list_callback(self, data):
        self.weed_list = data

    def odom_callback(self, data):
        self.odom = data
        odom = PoseStamped()
        odom.header = self.odom.header
        odom.pose = self.odom.pose.pose
        self.odom_map = self.tf_listener.transformPose('map', odom)

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
        
        self.move_client.send_goal(goal) # send new goal to actionlib server
       
        success = self.move_client.wait_for_result(rospy.Duration(90)) # wait up to 90s for result
       
        state = self.move_client.get_state() # get the state of the current goal
        result = False # create result bool and set it to false
       
        if success and state == GoalStatus.SUCCEEDED: # if success is true and state == succeeded
         result = True # set result as true
        else:
         self.move_client.cancel_goal() # if failure, cancle goal
       
        return result # return result
        

def main(args):
    '''Row Navigation Node'''
    rospy.init_node('RowNav', anonymous=True)
    rn = RowNav()
    rn.navigation_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        rn.stop_nav = True
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)