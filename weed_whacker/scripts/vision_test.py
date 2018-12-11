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

class image_converter:
    camera_model = None
    weeds = []

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect',
                                          Image, self.weeding)
	self.tf_listener = tf.TransformListener()

    def weeding(self, data): #weed finding in camera image

    	#set up hsv image from camera sub
    	cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
	#set mask
    	lower_green = np.array([40,20,20])
    	upper_green = np.array([80,255,255])        
    	image_mask = cv2.inRange(hsv, lower_green, upper_green)  
	#erode and open the image to split up plants
    	kernel = np.ones((3,3),np.uint8)
	kernel2 = np.ones((7,7),np.uint8)

	erode = cv2.erode(image_mask,kernel,iterations = 1)
    	dilation = cv2.dilate(erode,kernel2,iterations = 10)

	_, contours, hierarchy = cv2.findContours(dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	plants = []
	for c in contours:
		peri = cv2.arcLength(c, True)
		area = cv2.contourArea(c)
		if ((area / peri) >= 10.0):
			plants.append(c)
	for c in plants: #https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
		M = cv2.moments(c)
		cX = 0
		cY = 0
		if M["m00"] != 0:
 			cX = int(M["m10"] / M["m00"])
 			cY = int(M["m01"] / M["m00"])
		else:
 			cX, cY = 0, 0
		cv2.putText(dilation, "weed", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)

	
	#resize for visualisation
    	cv_image_s = cv2.resize(dilation, (0,0), fx=0.5, fy=0.5)

    	cv2.imshow("Image test", cv_image_s)
    	cv2.waitKey(1)

cv2.startWindowThread()
rospy.init_node('vis_test')
ic = image_converter()
rospy.spin()

cv2.destroyAllWindows()
