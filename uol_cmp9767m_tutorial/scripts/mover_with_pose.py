#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from math import cos, sin
from tf import TransformListener

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
        self.pose_pub = rospy.Publisher(
            '/nearest_obstacle',
            PoseStamped,queue_size=1
        )
        self.listener = TransformListener()


    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        # some logging on debug level, not usually displayed
        rospy.logdebug("I heard %s", data.header.seq)

        # This find the closest of all laser readings
        min_dist = min(data.ranges)

        # let's already create an object of type Twist to
        # publish it later. All initialised to 0!
        t = Twist()

        # If anything is closer than 4 metres anywhere in the
        # scan, we turn away
        if min_dist < 2:
            t.angular.z = 1.0
        else:  # if all obstacles are far away, let's keep 
            # moving forward at 0.8 m/s
            t.linear.x = 0.8
        # publish to the topic that makes the robot actually move
        self.publisher.publish(t)

        # above in min_dist we found the nearest value,
        # but to display the position of the nearest value
        # we need to find which range index corresponds to that
        # min_value.  
        # find the index of the nearest point 
        # (trick from https://stackoverflow.com/a/11825864)
        # it really is a very Python-ic trick here using a getter 
        # function on the range. Can also be done with a 
        # classic for loop
        index_min = min(
            range(len(data.ranges)),
            key=data.ranges.__getitem__)
        
        # convert the obtained index to angle, using the 
        # reported angle_increment, and adding that to the 
        # angle_min, i.e. the angle the index 0 corresponds to.
        # (is negative, in fact -PI/2).
        alpha = data.angle_min + (index_min * data.angle_increment)

        # No time for some trigonometry to turn the 
        # polar coordinates into cartesian coordinates
        # inspired by https://stackoverflow.com/a/20926435
        # use trigonometry to create the point in laser 
        # z = 0, in the frame of the laser 
        laser_point_2d = [ 
            cos(alpha) * min_dist, 
            sin(alpha) * min_dist,
            0.0]

        # create an empty PoseStamped to be published later.
        pose = PoseStamped()

        # keep the frame ID (the entire header here) as read from 
        # the sensor. This is general ROS practice, as it 
        # propagates the recording time from the sensor and 
        # the corrdinate frame of the sensor through to
        # other results we may be publishing based on the anaylysis
        # of the data of that sensor

        pose.header = data.header

        # fill in the slots from the points calculated above.
        # bit tedious to do it this way, but hey... 
        pose.pose.position.x = laser_point_2d[0]
        pose.pose.position.y = laser_point_2d[1]
        pose.pose.position.z = laser_point_2d[2]

        # using my trick from https://github.com/marc-hanheide/jupyter-notebooks/blob/master/quaternion.ipynb
        # to convert to quaternion, so that the Pose always 
        # points in the direction of the laser beam. 
        # Not required if only the positions is
        # of relevance 
        # (then just set pose.pose.orientation.w = 1.0 and leave
        # others as 0).
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(alpha/2)
        pose.pose.orientation.w = cos(alpha/2)

        # publish the pose so it can be visualised in rviz
        self.pose_pub.publish(pose)

        rospy.loginfo(
            "The closest point in laser frame coords is at\n%s"
            % pose.pose.position
            )

        # now lets actually transform this pose into a robot 
        # "base_link" frame.
        transformed_pose = self.listener.transformPose("thorvald_001/base_link", pose)
        rospy.loginfo(
            "The closest point in robot coords is at\n%s"
            % transformed_pose.pose.position
            )



if __name__ == '__main__':
    # as usual, initialise the ROS node with a name
    rospy.init_node('mover')
    # Create the Mover object 
    # (which in turns registers the subscriber and make the system go)
    Mover()
    # Finally, keep going until we are interrupted.
    rospy.spin()

