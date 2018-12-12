#!/usr/bin/env python
import sys
import time
import numpy as np

import rospy
import image_geometry
import roslib
import tf

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from simple_move_base import Go_To_Point
from std_msgs.msg import Bool

class image_converter:
    camera_model = None
    weeds = []
    lock = True
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect',
                                          Image, self.image_callback)
	self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            				  CameraInfo, self.camera_info_callback)
	self.go_sub = rospy.Subscriber('/find_row', 
            				  Bool, self.search)
	self.tf_listener = tf.TransformListener()
	self.curr_image = Image()

    def camera_info_callback(self, data): #get camera info once
	self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()

    def image_callback(self, data):
	self.curr_image = data
	self.lock = False

    def search(self, data): #row finding in camera image
	if data == False:
		return
	if self.lock == True:
		return
	(trans, rot) = self.tf_listener.lookupTransform('map', 
            'thorvald_001/kinect2_rgb_optical_frame', rospy.Time())

    	#set up hsv image from camera sub
    	cv_image = self.bridge.imgmsg_to_cv2(self.curr_image, "bgr8")
    	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
	#set mask
    	lower_green = np.array([40,20,20])
    	upper_green = np.array([80,255,255])        
    	image_mask = cv2.inRange(hsv, lower_green, upper_green)  
	#erode and open the image to split up plants
    	kernel = np.ones((3,3),np.uint8)
	kernel2 = np.ones((7,7),np.uint8)

	erode = cv2.erode(image_mask,kernel,iterations = 1)
    	dilation = cv2.dilate(erode,kernel2,iterations = 1)

	_, contours, hierarchy = cv2.findContours(dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	
	contours = sorted(contours, key=cv2.contourArea)

	rows = []
	for c in range(len(contours)):
		x,y,w,h = cv2.boundingRect(contours[c])
		x = x+w
		y = y+(h/2)
		ray = self.camera_model.projectPixelTo3dRay((x,y))

		p_point = PoseStamped()
		p_point.header.frame_id = 'thorvald_001/kinect2_rgb_optical_frame'
		p_point.pose.orientation.w = 1.0
		p_point.pose.position.x = ray[0]
		p_point.pose.position.y = ray[1]
		p_point.pose.position.z = ray[2]
		p_test = self.tf_listener.transformPose('map', p_point)

		vector = [p_test.pose.position.x - trans[0],
		  	  p_test.pose.position.y - trans[1],
		  	  p_test.pose.position.z - trans[2]]
		vector_n = []
		for c in range(3):
			vector_n.append(vector[c] / ((vector[0]*vector[0])+(vector[1]*vector[1])+(vector[2]*vector[2]))**.5)
		change_mag = (0.0 - p_test.pose.position.z)/vector_n[2]
		inter_vec = [p_test.pose.position.x + (vector_n[0]*change_mag),p_test.pose.position.y + (vector_n[1]*change_mag),0]
		rows.append((inter_vec[0], inter_vec[1]))

	row_dist = []
	for r in range(len(rows)):
		dist = (((rows[r][0]-trans[0])*(rows[r][0]-trans[0]))+((rows[r][1]-trans[1])*(rows[r][0]-trans[0])))**0.5
		row_dist.append(dist)
	nav_lock = False
	if len(row_dist) != 0:
		_,coord = min((row_dist[i],i) for i in xrange(len(row_dist)))
		if row_dist[coord] < 2:
			nav_lock = True
			print "turn on weed finding" #post back to control https://answers.ros.org/question/69754/quaternion-transformations-in-python/ !!!!!!!!!! (orient bot)
		print row_dist[coord]
		go_place = []
		go_place.append(rows[coord][0])#x
		go_place.append(rows[coord][1])#y
		go_place.append(0)#ox
		go_place.append(0)#oy
		go_place.append(0)#oz
		move_base_commander = Go_To_Point()
		x = 0.5
		success = move_base_commander.point(go_place, x) #go_place = location for sprayer, 20 = time(s) until goal is rejected

		if success:
			print "bam!"
		else:
			print "no bam :( "
	else:
		print "finished"
rospy.init_node('vis_test')
ic = image_converter()
rospy.spin()
