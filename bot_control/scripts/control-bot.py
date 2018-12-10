#!/usr/bin/env python

import rospy
import time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# def initialize():
#     global acc_error
#     global prev_error
#     acc_error = 0
#     prev_error = 0

# def current_target_loc():
    # distance = math.sin((current_milli_time()%314159)/70000

def bangBang(error):
    move = Twist()
    move.linear.x = 0
    if error > 0.05:
        move.linear.x = 1
    if error < -0.05:
        move.linear.x = -1
    pub.publish(move)

def pControl(error):
    move = Twist()
    move.linear.x = 0
    if abs(error) > 0.05:
        move.linear.x = 0.6*error
    pub.publish(move)

def piControl(error):
    global acc_error
    try:
        acc_error
    except NameError:
        acc_error = 0
    acc_error += error
    # print(acc_error)
    move = Twist()
    move.linear.x = 0
    if abs(error) > 0.05:
        move.linear.x = 0.6*error + 0.001*acc_error
        # print(move.linear.x)
    pub.publish(move)

def pidControl(error):
    global acc_error
    global prev_error
    try:
        acc_error
    except NameError:
        acc_error = 0
    try:
        prev_error
    except NameError:
        prev_error = 0
    acc_error += error
    # print(acc_error)
    move = Twist()
    move.linear.x = 0
    if abs(error) > 0.05:
        move.linear.x = 0.6*error + 0.001*acc_error + 0.6*(error-prev_error)
    pub.publish(move)
    prev_error = error

def callback(data):
    front = data.ranges[360]

    error = front - 7

    # bangBang(error)
    # pControl(error)
    piControl(error)
    # pidControl(error)
    # left = data.ranges[0]
    # right = data.ranges[719]
    # reverse = False    
    
    # move = Twist()
    # if (front < 2):
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

if __name__ == '__main__':
    # initialize()
    current_milli_time = lambda: int(round(time.time() * 1000))
    rospy.init_node('botrun')
    pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)
    # rate = rospy.Rate(10)
    rospy.Subscriber('/thorvald_001/scan', LaserScan, callback)
    rospy.spin()
