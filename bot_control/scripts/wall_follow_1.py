#!/usr/bin/env python

import rospy
import time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# def initialize():
#     global acc_error
#     global prev_error
#     acc_error = 0
#     prev_error = 0

# def current_target_loc():
    # distance = math.sin((current_milli_time()%314159)/70000

def turn_left():
    pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)
    move = Twist()
    move.angular.z = -2
    init_angle = odom_data.pose.pose.orientation.z
    print(init_angle)
    print(str(odom_data.pose.pose.orientation.z + 1.571))
    print(move)
    while init_angle < (odom_data.pose.pose.orientation.z + 1.571):
        pub.publish(move)

def lidar_callback(data):
    lidar_data = data

def odom_callback(data):
    # print(data.pose.pose.orientation.z)
    odom_data = data
    turn_left()

# def runbot():
#     front = lidar_data.ranges[360]
#     left = lidar_data.ranges[0]
#     right = lidar_data.ranges[719]
     
#     move = Twist()
    
    # if (front < 1):
    #     turn_left(move)
    #     move.linear.x = -10
    #     if (left > right):
    #         move.angular.z = 2
    #     else:
    #         move.angular.z = -2
    # else:
    #     move.linear.x = front*2
    # if (front < 7.0):
    #     move.angular.z = 2
    # if (right < 2.0):
    #     move.angular.z = 2
    # if (left < 2.0):
    #     move.angular.z = -2
    #    if (left > right):
    #        move.angular.z = 3/left
    #    else:
    #        move.angular.z = -3/right
    # pub.publish(move)

def main():
    # initialize()
    current_milli_time = lambda: int(round(time.time() * 1000))
    rospy.init_node('botrun')
    # pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)
    # rate = rospy.Rate(10)
    global lidar_data
    lidar_data = LaserScan()
    global odom_data
    odom_data = Odometry()
    
    rospy.Subscriber('/thorvald_001/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/thorvald_001/odometry/gazebo', Odometry, odom_callback)
    rospy.spin()
    # print("Got data")
    # move = Twist()
    # turn_left(move, pub)
    # print("stopped")
    # while(True):
    #     if lidar_data.ranges.__len__() == 720:
    #         runbot()
    #     time.sleep(20)

if __name__ == '__main__':
    main()
