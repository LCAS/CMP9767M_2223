#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover:
    """
    A very simple Roamer implementation for Thorvald.
    It simply goes straight until any obstacle is within
    3 m distance and then just simply turns left.
    A purely reactive approach.
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback)

    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        rospy.loginfo(
            rospy.get_caller_id() + "I heard %s", data.header.seq)
        min_dist = min(data.ranges)
        t = Twist()
        if min_dist < 4:
            t.angular.z = 1.0
        else:
            t.linear.x = 0.8
        self.publisher.publish(t)

if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

