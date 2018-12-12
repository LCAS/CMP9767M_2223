#!/usr/bin/env python

import rospy
import cv2
import numpy
from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Empty

#ACTIVATED 1ST
class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)
        self.sprayer =  rospy.ServiceProxy("/thorvald_001/spray", Empty)
        self.ind_sub = rospy.Subscriber("/stage_indicator", Int8, self.ind_callback)

    def callback(self, data):
        pub = rospy.Publisher("/colorval_camera", Float32, queue_size=1)
        #Creating Output Windows
        cv2.namedWindow("Final")
        cv2.namedWindow("Mask")
        cv2.namedWindow("Circles")
        #Formatting Image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        output = cv_image.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #Setting mask colour range
        lower_green = numpy.array([36,0,0])
        upper_green = numpy.array([60,255,255])
        #creating mask
        mask = cv2.inRange(hsv, lower_green, upper_green)
        #finding weeds through shape then drawing circle on found circles
        #wont always only pick up weeds cause of plant shapes it will find circles in soil
        circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1.05,150,
            param1=30,
            param2=15,
            minRadius=50,
            maxRadius=125)
        #prints circles only if centre is on green background   
        try: 
            circles = numpy.round(circles[0,:]).astype("int")
            for (x, y, r) in circles:
                #only for circles with centre on a plant
                if mask[y, x] != 0:
                    cv2.circle(output,(x, y), r, (0, 0, 255), 4)
                    cv2.rectangle(output, (x - 2, y - 2), (x + 2, y + 2), (0, 128, 255), -1)
                    print([x, y])
                    if x > 1600:
                        self.sprayer()
        except TypeError:
            pub.publish(float(0))     #outputs 0 if no circles are found
        #combining circles and mask to see the circles seen in plant    
        res = cv2.bitwise_and(output,output, mask= mask)
        colorval = numpy.mean(res)
        pub.publish(colorval)
        cv2.imshow("Mask", mask)
        cv2.imshow("Circles", output)
        cv2.imshow("Final", res)
        cv2.waitKey(1)

    def ind_callback(self, data):
        ind_pub = rospy.Publisher("stage_indicator", Int8, queue_size=1)
        ind_pub.publish(0)
        if data.data != 0:
            ind_pub.publish(data.data)      
        
        
cv2.startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()
cv2.destroyAllWindows()