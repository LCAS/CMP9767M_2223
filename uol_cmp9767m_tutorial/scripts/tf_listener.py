#!/usr/bin/env python  
import rospy

import math
import tf
import geometry_msgs.msg
from math import atan2, pi


class MyTFListener:
    """
    A simple node showing the use of a TF Listener.
    Requires the simulation to be up and running with two robots for
    didactic purposes, including fake localisation to establish the transformation
    between the two robots. Launch it as 
    `roslaunch uol_cmp9767m_base thorvald-sim.launch second_robot:=true fake_localisation:=true`
    """

    def __init__(self):
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('test_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

        self.rate = rospy.Rate(.5)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # look up the transform, i.e., get the transform from Thorvald_001/base_link to the world,
                # This transform will allow us to transfer 
                (trans, rot) = self.listener.lookupTransform('world', 'thorvald_001/base_link', rospy.Time())
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.rate.sleep()
                print(e)
                continue

            # the quick hack to go from quaternions to 2D angles:
            yaw_angle = atan2(rot[2], rot[3])*2
            print("the current transformation: ", trans, yaw_angle * 180 / pi)
            
            # here is an exmaple pose, with a given frame of reference, e.g. somethng detected in the camera
            p1 = geometry_msgs.msg.PoseStamped()
            # every pose is given in relation to a frame, indicated here as frame_id
            p1.header.frame_id = "thorvald_001/kinect2_left_rgb_optical_frame"
            # Remember the quaternion (0,0,0,1) is no rotation.
            p1.pose.orientation.w = 1.0  # Neutral orientation 

            p1.pose.position.z = 1  # a metre away from the frame centre (a metre in front of camera)
            # we publish this so we can see it in rviz:   
            self.pose_pub.publish(p1)

            # here we directly transform the pose into another pose for the given frame of reference:
            p_in_base = self.listener.transformPose("thorvald_001/base_link", p1)
            print "Position of the object in the new frame of reference:"
            print p_in_base

            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('tf_listener')
    t = MyTFListener()
    t.run()

