#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from math import cos, sin

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
        rospy.Subscriber("/thorvald_001/scan", LaserScan, self.callback)
        self.pose_pub = rospy.Publisher(
            '/nearest_obstacle',
            PoseStamped,queue_size=1
        )

    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        rospy.loginfo(
            rospy.get_caller_id() + "I heard %s", data.header.seq)
        min_dist = min(data.ranges)
        t = Twist()
        if min_dist < 3:
            t.angular.z = 1.0
        else:
            t.linear.x = 2.0
        self.publisher.publish(t)

        # find the nearest point 
        # (trick from https://stackoverflow.com/a/11825864)
        index_min = min(
            range(len(data.ranges)),
            key=data.ranges.__getitem__)
        
        #convert to angle
        alpha = data.angle_min + (index_min * data.angle_increment)
        point = [ # use trigonometry to create the point in laser frame
            cos(alpha) * min_dist, 
            sin(alpha) * min_dist,
            0.0]

        # create an emoty PoseStamped to be published later.
        pose = PoseStamped()

        # keep the frame ID (the entire header here) as read from the sensor
        pose.header = data.header

        # fill in the slots from the points calculated above
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]

        # using my trick from https://github.com/marc-hanheide/jupyter-notebooks/blob/master/quaternion.ipynb
        # to convert to quaternion, so that the Pose always points in the 
        # direction of the laser beam. Not required if only the positions is
        # of relevance (then just set pose.pose.orientation.w = 1.0 and leave
        # others as 0).
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(alpha/2)
        pose.pose.orientation.w = cos(alpha/2)

        # publish the pose so it can be visualised in rviz
        self.pose_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

