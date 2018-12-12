#!/usr/bin/env python

import rospy
#import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class 
def Callback(data):
    pub = rospy.Publisher("thorvald_001/twist_mux/cmd_vel", Twist, queue_size=1)
    t = Twist()
    if data.ranges[360] < 2:
        t.linear.x = 0
        Callback2()
                
def Callback2(data):
    




def listener():
        rospy.init_node('listener', anonymous=True, disable_signals=True)
        rospy.Subscriber = rospy.Subscriber("/thorvald_001/scan", LaserScan, callback)
        rospy.Subscriber2 = rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, callback2)
        rospy.spin()
        
  self.camera_info_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/camera_info", CameraInfo, self.coord_callback)
        
        self.tf_listener = tf.TransformListener()

    def coord_callback(self, data):
        if not self.camera_model:
            return
        (trans, rot) = self.tf_listener.lookupTransform("thorvald_001/base_link", "thorvald_001/kinect2_rgb_optical_frame", rospy.Time())
        print 'Robot to camera transform:', 'T ', trans, 'R ', rot
        
        p_robot = PoseStamped()
        p_robot.header.frame_id = "thorvald_001/base_link"
        p_robot.pose.orientation.w = 1.0
        
        p_robot.pose.position.x = 0.45
        p_robot.pose.position.y = 0.0
        p_robot.pose.position.y = 0.0
        p_camera = self.tf_listener.transformPose("thorvald_001/kinect2_rgb_optical_frame", p_robot)
        print 'Point in the camera coordinates'
        print p_camera.pose.position
        
        uv = self.camera_model.project3dToPixel((p_camera.pose.position.x,p_camera.pose.position.y,p_camera.pose.position.z))
        
        print 'Pixel coordinates: ', uv
        print ''
        
         print(len(data.ranges))
    val, idx = min((val, idx) for (idx, val) in enumerate(data.ranges))
    print (val)
    print (idx)
    if val > 1.0:
        diffv = abs(val-1.0)
        t.linear.x = 0.2*(diffv)
        pub.publish(t)      
    else:
        t.linear.x = 0.0
        pub.publish(t)
        diffa = abs(360-idx)
        if idx > 360:
            t.angular.z = ((math.pi*(diffa**2)*0.0001))/18
            pub.publish(t)
        else:
            if idx < 360:
                t.angular.z = -((math.pi*(diffa**2)*0.0001))/18
                pub.publish(t) 
            else:
                t.angular.z = 0
                rospy.signal_shutdown("wall found")
