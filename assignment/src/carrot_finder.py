#!/usr/bin/env python

import rospy
from cv2 import namedWindow, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey, morphologyEx, MORPH_CLOSE
from cv2 import threshold, THRESH_BINARY, split, medianBlur, connectedComponents
from cv2 import merge, cvtColor, COLOR_HSV2BGR
from numpy import ones, uint8, max, ones_like
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
        #OPENCV DISPLAY REMOVED FOR INCREASED PERFORMANCE
        #namedWindow("Image window")

        #defining publisher for output image
        camera_objects = rospy.Publisher("camera_objects", Image, queue_size = 1)  

        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #splitting image into b,g,r channels        
        b,g,r = split(cv_img)        
        #performing binary thresholding operation on green channel
        ret, thresh = threshold(g, 50, 255, THRESH_BINARY)
        
        #consider replacing with opening, and increasing kernel size to further remove small objects/noise
        #preforming median filter to remove noise
        median_result = medianBlur(thresh, 11)
        
        #performing closing operation to remove voids in remaining objects
        closed_img = morphologyEx(median_result, MORPH_CLOSE, self.kernel)
        
        #finding the connected components from the closed image, Connected Components Example adapted from A.Reynolds (2017) Source: https://stackoverflow.com/questions/46441893/connected-component-labeling-in-python?rq=1
        retval, labels = connectedComponents(closed_img)        
        
        #iterating through the labels to find carrot/non-carrot objects
        
        # Map component labels to hue val
        label_hue = uint8(179*labels/max(labels))
        blank_ch = 255*ones_like(label_hue)
        labeled_img = merge([label_hue, blank_ch, blank_ch])

        # cvt to BGR for display
        labeled_img = cvtColor(labeled_img, COLOR_HSV2BGR)

        # set bg label to black
        labeled_img[label_hue==0] = 0
        

        #displaying the result of thresholding, and closing operations
        #OPENCV DISPLAY REMOVED FOR INCREASED PERFORMANCE#imshow("Image window", labeled_img)
        ##END DISPLAY METHODS        
        
        #publishing image
        output = self.bridge.cv2_to_imgmsg(labeled_img, encoding="rgb8")
        camera_objects.publish(output)

        waitKey(1)

startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()