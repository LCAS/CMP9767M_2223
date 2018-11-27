#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class kinect_processor:
    camera_model = None

    def __init__(self):    

        self.bridge = CvBridge()
        self.kinect_processed_pub = rospy.Publisher('/thorvald_001/kinect_processed_pub', 
            Image, queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.image_callback)

        self.tf_listener = tf.TransformListener()

    def image_callback(self, data):
        if not self.camera_model:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        blur = cv2.medianBlur(img, 5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_blur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        res,thresh = cv2.threshold(hsv[:,:,0], 55, 255, cv2.THRESH_BINARY)
        res,thresh2 = cv2.threshold(hsv[:,:,2], 140, 255, cv2.THRESH_BINARY)
        res,thresh3 = cv2.threshold(hsv_blur[:,:,0], 45, 255, cv2.THRESH_BINARY)

        final = cv2.bitwise_and(thresh, thresh2)
        kernel = np.ones((8,8), 'uint8')
        dilate = cv2.dilate(thresh3, kernel, iterations=4)
        # kernel = np.ones((5,5), 'uint8')
        # dilate = cv2.dilate(thresh3, kernel, iterations=2)
        edges = cv2.Canny(dilate, 75, 150)

        lines = cv2.HoughLinesP(edges, 4, np.pi/180, 90, minLineLength=150, maxLineGap=50)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(blur, (x1, y1), (x2, y2), (0, 255, 0), 4)

        _, contours, hierarchy = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        img2 = img.copy()
        index = -1
        thickness = 1
        color = (255, 0, 0)

        filter_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0 or area == 0 or perimeter == 0:
                continue
            solidity = area/hull_area
            if solidity < 0.7 or area < 1500:
                continue
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int( M['m10']/M['m00'])
                cy = int( M['m01']/M['m00'])
                cv2.circle(img2, (cx,cy), 2, (0,0,255), -1)
                cv2.putText(img2, str(area)+" "+str(round(solidity,3)),(cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255,255,255),1,cv2.LINE_AA)
            else:
                print(M)
            filter_contours.append(c)
            # cv2.drawContours(objects, [c], -1, color, -1)
            # print("Area: {}, Perimeter: {}, Hull Area: {}, Solidity: {}".format(area, perimeter, hull_area, solidity))

        cv2.drawContours(img2, filter_contours, index, color, thickness)

        try:
            ros_img = self.bridge.cv2_to_imgmsg(img2, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.kinect_processed_pub.publish(ros_img)
        

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

def main(args):
    '''Kinect Processor node'''
    rospy.init_node('kinect_processor', anonymous=True)
    kp = kinect_processor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)