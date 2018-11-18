#!/usr/bin/env python

import rospy
from cv2 import namedWindow, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey, morphologyEx, MORPH_CLOSE
from cv2 import threshold, THRESH_BINARY, split, medianBlur
from numpy import zeros, ones, uint8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        #initialising subscriber
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
                                          Image, self.callback)
        #initialising kernel for closing operation                                   
        self.kernel = ones((5,5), uint8)                                  

    def callback(self, data):
        namedWindow("Image window")

        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #splitting image into b,g,r channels        
        b,g,r = split(cv_img)        
        #performing binary thresholding operation on green channel
        ret, thresh = threshold(g, 50, 255, THRESH_BINARY)
        
        #preforming median filter to remove noise
        median_result = medianBlur(thresh, 5)
        
        #performing closing operation to remove voids in remaining objects
        closed_img = morphologyEx(median_result, MORPH_CLOSE, self.kernel)
        
        #displaying the result of thresholding, and closing operations
        imshow("Image window", closed_img)
                
        
        waitKey(1)

startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()