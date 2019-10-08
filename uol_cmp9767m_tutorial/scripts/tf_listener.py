#!/usr/bin/env python  
import rospy

import math
import tf
import geometry_msgs.msg
from math import atan2, pi

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('test_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # look up the transform, i.e., get the transform from Thorvald_002 to Thorvald_001,
            # This transform will allow us to transfer 
            (trans, rot) = listener.lookupTransform('thorvald_001/base_link', 'thorvald_002/base_link', rospy.Time())
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        yaw_angle = atan2(rot[2], rot[3])*2
        print("the current transformation: ", trans, yaw_angle * 180 / pi)
        
        # here is an exmaple pose, with a given frame of reference, e.g. somethng detected in the camera
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "thorvald_002/kinect2_rgb_optical_frame"
        p1.pose.orientation.w = 1.0  # Neutral orientation 
        p1.pose.position.z = .5  # half a metre away from the from frame centre
        # we publish this so we can see it in rviz:   
        pose_pub.publish(p1)

        # here we directly transform the pose into another pose for the given frame of reference:
        p_in_base = listener.transformPose("thorvald_001/base_link", p1)
        print "Position of the object in the new frame of reference:"
        print p_in_base



        rate.sleep()