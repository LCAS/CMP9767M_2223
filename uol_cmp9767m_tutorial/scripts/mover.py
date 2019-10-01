#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover:

    def __init__(self):
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/scan", LaserScan, self.callback)

    def callback(self, data):

        rospy.loginfo(
            rospy.get_caller_id() + "I heard %s", data.header.seq)
        min_dist = min(data.ranges)
        t = Twist()
        if min_dist < 3:
            t.angular.z = 1.0
        else:
            t.linear.x = 1.0
        self.publisher.publish(t)

if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

