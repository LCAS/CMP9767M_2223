#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np
import math

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

class KinectProcessor:

    def __init__(self):    

        self.bridge = CvBridge()
        self.dist_coeff_x = 84.1/1920.0
        self.dist_coeff_y = 53.8/1080.0
        self.degree_diff = 45.0 - (84.1/2)
        self.kinect_height = 0.5
        self.deg2rad = math.pi/180
        self.kinect_vert_half = 1080/2
        self.kinect_x_disp = 0.45
        self.mode = 2 # 0 = Find field, 1 = align with row

        # transform listener
        self.tf_listener = tf.TransformListener()

        # Processed Image Publishers: optional, may be turned off with /show_img param
        self.kinect_processed_pub = rospy.Publisher('/thorvald_001/kinect_processed_pub', 
            Image, queue_size=1)
        self.kinect_processed_thresh1 = rospy.Publisher('/thorvald_001/kinect_processed_thresh1', 
            Image, queue_size=1)
        # self.kinect_processed_thresh2 = rospy.Publisher('/thorvald_001/kinect_processed_thresh2', 
        #     Image, queue_size=1)
        self.kinect_processed_final = rospy.Publisher('/thorvald_001/kinect_processed_final', 
            Image, queue_size=1)
        
        # Weed 3d point: Published the weed's coordinates in the map frame
        self.weed_location_pub = rospy.Publisher('/thorvald_001/weed_location', 
            PointStamped, queue_size=1)

        # Subscriber for kinect rgb cam
        rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.image_callback)
        
    # callback for subscriber to kinect rgb cam.
    def image_callback(self, data):
        # try converting rgbcam ros message to cv2 image, return if failed
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        # check for the show_img ros param. If true, the processed images are published
        if rospy.has_param('show_img') and rospy.get_param('show_img') == True:
            self.show_img = True
        else:
            self.show_img = False
        
        if self.mode == 0:
            self.find_field(img)
        elif self.mode == 1:
            self.find_weeds(img)
        elif self.mode == 2:
            self.find_weeds(img)
    #
    # finds field in the image
    #
    def find_field(self, img):
        # convert BGR to HSV 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # filter Hue for green color
        res,thresh1 = cv2.threshold(hsv[:,:,0], 60, 255, cv2.THRESH_BINARY)
        # filter Value for bright regions in image
        # res,thresh2 = cv2.threshold(hsv[:,:,2], 140, 255, cv2.THRESH_BINARY)

        # a 4x4 kernel to be used for morphing
        kernel = np.ones((10,10), 'uint8')
        # Open expands the area
        final = cv2.morphologyEx(thresh1, cv2.MORPH_CLOSE, kernel, iterations=4)

        # bitwise AND on thresh1 and thresh2 to get common regions.
        # final = cv2.bitwise_and(thresh1, thresh2)
        # blur final to straigten out the jagged edges

        if self.show_img:
            img2 = img.copy()
            index = -1
            thickness = 1
            color = (255, 0, 0)
        
        # check if we're allowed to publish the processed images.
        if self.show_img:
            try:
                # draw an outline
                # cv2.drawContours(img2, filter_contours, index, color, thickness)

                # convert cv2 image to ros message image. Binary images use the default passthrough
                ros_img1 = self.bridge.cv2_to_imgmsg(thresh1, encoding="passthrough")
                ros_img3 = self.bridge.cv2_to_imgmsg(final, encoding="passthrough")
                ros_img4 = self.bridge.cv2_to_imgmsg(img2, "bgr8")

                # publish images through respective Publishers
                ros_img1.header.stamp = rospy.Time.now()
                ros_img3.header.stamp = rospy.Time.now()
                ros_img4.header.stamp = rospy.Time.now()
                self.kinect_processed_thresh1.publish(ros_img1)
                self.kinect_processed_final.publish(ros_img3)
                self.kinect_processed_pub.publish(ros_img4)
            except CvBridgeError as e:
                print(e)
                return

    #
    # finds weeds in the image and published their map cordinates
    #
    def find_weeds(self, img):
        # convert BGR to HSV 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # filter Hue for green color
        res,thresh1 = cv2.threshold(hsv[:,:,0], 55, 255, cv2.THRESH_BINARY)
        # filter Value for bright regions in image
        # res,thresh2 = cv2.threshold(hsv[:,:,2], 140, 255, cv2.THRESH_BINARY)

        # a 4x4 kernel to be used for morphing
        kernel = np.ones((4,4), 'uint8')
        # 4 passes of erosion on final.
        # Removes smaller regions, breaks up thin stems to extract individual leaves
        final = cv2.erode(thresh1, kernel, iterations=3)
        final = cv2.morphologyEx(final, cv2.MORPH_OPEN, kernel, iterations=3)

        # bitwise AND on thresh1 and thresh2 to get common regions.
        # final = cv2.bitwise_and(thresh1, thresh2)
        # blur final to straigten out the jagged edges
        final = cv2.medianBlur(final, 5)
        res, final = cv2.threshold(final, 200, 255, cv2.THRESH_BINARY)

        # find contours from final binary image
        _, contours, hierarchy = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if self.show_img:
            img2 = img.copy()
            index = -1
            thickness = 1
            color = (255, 0, 0)
        # contour filter, stores contours that qualify as weeds
        filter_contours = []
        # iterate through contours to find weeds
        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0 or area == 0 or perimeter == 0:
                continue
            solidity = area/hull_area
            if solidity < 0.85 or area < 3000:
                continue

            M = cv2.moments(c)
            if M['m00'] != 0:
                weed_x = int( M['m10']/M['m00'])
                weed_y = int( M['m01']/M['m00'])
                
                pt_kinect = PointStamped()
                pt_kinect.header.stamp = rospy.Time(0)
                # pt_kinect.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"
                pt_kinect.header.frame_id = "thorvald_001/base_link"
                pt_kinect.point.x = self.kinect_height * math.tan(
                    self.deg2rad * (self.degree_diff + self.dist_coeff_x * weed_x))

                if (pt_kinect.point.x * math.tan( 1.1 * self.deg2rad * 
                    (self.kinect_vert_half - math.sqrt(area)) * self.dist_coeff_y)) < 0.07:
                    continue
                pt_kinect.point.y = pt_kinect.point.x * math.tan( 1.3 *
                    self.deg2rad * (self.kinect_vert_half - weed_y) * self.dist_coeff_y)

                pt_kinect.point.z = 0
                pt_kinect.point.x += self.kinect_x_disp + 0.45
                try:
                    weed_coords = self.tf_listener.transformPoint('map', pt_kinect)
                except tf.ExtrapolationException:
                    continue
                weed_coords.point.z = 0
                weed_coords.header.stamp = rospy.Time.now()
                if self.show_img:
                    filter_contours.append(c)
                self.weed_location_pub.publish(weed_coords)

                if self.show_img:
                    cv2.circle(img2, (weed_x,weed_y), 2, (0,0,255), -1)
                    cv2.putText(img2, str(weed_coords.point.x)+" "+str(weed_coords.point.y)
                        +" "+str(weed_coords.point.z)+" "+str(round(solidity,3)),(weed_x,weed_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255),1,cv2.LINE_AA)
            else:
                print(M)

        # check if we're allowed to publish the processed images.
        if self.show_img:
            try:
                # draw an outline of detected weeds
                cv2.drawContours(img2, filter_contours, index, color, thickness)

                # convert cv2 image to ros message image. Binary images use the default passthrough
                ros_img1 = self.bridge.cv2_to_imgmsg(thresh1, encoding="passthrough")
                # ros_img2 = self.bridge.cv2_to_imgmsg(thresh2, encoding="passthrough")
                ros_img3 = self.bridge.cv2_to_imgmsg(final, encoding="passthrough")
                ros_img4 = self.bridge.cv2_to_imgmsg(img2, "bgr8")

                # publish images through respective Publishers
                ros_img1.header.stamp = rospy.Time.now()
                # ros_img2.header.stamp = rospy.Time.now()
                ros_img3.header.stamp = rospy.Time.now()
                ros_img4.header.stamp = rospy.Time.now()
                self.kinect_processed_thresh1.publish(ros_img1)
                # self.kinect_processed_thresh2.publish(ros_img2)
                self.kinect_processed_final.publish(ros_img3)
                self.kinect_processed_pub.publish(ros_img4)
            except CvBridgeError as e:
                print(e)
                return


def main(args):
    '''Kinect Processor node'''
    rospy.init_node('kinect_processor', anonymous=True)
    kp = KinectProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)