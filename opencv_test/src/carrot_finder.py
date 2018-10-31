#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny, threshold, THRESH_BINARY
from numpy import median
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
                                          Image, self.callback)

    def callback(self, data):
        namedWindow("Image window")

        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_img = cvtColor(cv_img, COLOR_BGR2GRAY)

        retval, plants = threshold(gray_img,50, 255, THRESH_BINARY)

        imshow("Image window", plants)
        
        #slice out the green channel, and do the thing
        
        
        waitKey(1)

startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()