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
    camera_model = None
    depth_img = None
    start_time = None

    def __init__(self):    

        self.bridge = CvBridge()


        # Processed Image Publishers: optional, may be turned off with /show_img param
        self.kinect_processed_pub = rospy.Publisher('/thorvald_001/kinect_processed_pub', 
            Image, queue_size=1)
        self.kinect_processed_thresh1 = rospy.Publisher('/thorvald_001/kinect_processed_thresh1', 
            Image, queue_size=1)
        self.kinect_processed_thresh2 = rospy.Publisher('/thorvald_001/kinect_processed_thresh2', 
            Image, queue_size=1)
        self.kinect_processed_final = rospy.Publisher('/thorvald_001/kinect_processed_final', 
            Image, queue_size=1)
        
        # Weed 3d point: Published the weed's coordinates in the map frame
        self.weed_location_pub = rospy.Publisher('/thorvald_001/weed_location', 
            PointStamped, queue_size=1)

        # Subscribers for kinect
        # Kinect rgb sensor data is 420 pixels wider than depth sensor data
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.image_callback)
        rospy.Subscriber("/thorvald_001/kinect2_sensor/sd/image_depth_rect",
            Image, self.depth_callback)

        self.tf_listener = tf.TransformListener()
        
    # callback for subscriber to kinect depth cam. Sets depth_img
    def depth_callback(self, data):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # callback for subscriber to kinect rgb cam.
    def image_callback(self, data):
        # check if camera_model is set
        if not self.camera_model:
            return
        # check if depth_img is set. If not, return
        try:
            _ = self.depth_img.shape
        except NameError:
            return
        except AttributeError:
            return
        # try converting rgbcam ros message to cv2 image, return if failed
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        # if start_time is not set, initialize it with current time
        # if not self.start_time:
        #     self.start_time = rospy.Time.now()
        # check for the show_img ros param. If true, the processed images are published
        if rospy.has_param('show_img') and rospy.get_param('show_img') == True:
            self.show_img = True
        else:
            self.show_img = False
        # convert BGR to HSV 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # filter Hue for green color
        res,thresh1 = cv2.threshold(hsv[:,:,0], 55, 255, cv2.THRESH_BINARY)
        # filter Value for bright regions in image
        res,thresh2 = cv2.threshold(hsv[:,:,2], 140, 255, cv2.THRESH_BINARY)

        # a 4x4 kernel to be used for morphing
        kernel = np.ones((4,4), 'uint8')
        # 4 passes of erosion on thresh2.
        # Removes smaller regions, breaks up thin stems to extract individual leaves
        thresh2 = cv2.erode(thresh2, kernel, iterations=4)

        # bitwise AND on thresh1 and thresh2 to get common regions.
        final = cv2.bitwise_and(thresh1, thresh2)
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
            if solidity < 0.7 or area < 2500:
                continue

            if self.show_img:
                filter_contours.append(c)
            M = cv2.moments(c)
            if M['m00'] != 0:
                weed_x = int( M['m10']/M['m00'])
                weed_y = int( M['m01']/M['m00'])
                depth_xy = self.coords_rgb2depth((weed_x,weed_y))
                if depth_xy[0] == -1:
                    continue
                dist = self.depth_img[depth_xy[0], depth_xy[1]]
                weed_cam = self.camera_model.projectPixelTo3dRay((weed_x,weed_y))
                weed_cam = [x * dist for x in weed_cam]

                pt_kinect = PointStamped()
                pt_kinect.header.stamp = rospy.Time(0)
                pt_kinect.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"
                pt_kinect.point.x = weed_cam[0]
                pt_kinect.point.y = weed_cam[1]
                pt_kinect.point.z = weed_cam[2]

                weed_coords = self.tf_listener.transformPoint('map', pt_kinect)
                weed_coords.point.z = 0
                if math.isnan(weed_coords.point.x) or math.isnan(weed_coords.point.y):
                    print(dist, depth_xy)
                    continue
                
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
                ros_img2 = self.bridge.cv2_to_imgmsg(thresh2, encoding="passthrough")
                ros_img3 = self.bridge.cv2_to_imgmsg(final, encoding="passthrough")
                ros_img4 = self.bridge.cv2_to_imgmsg(img2, "bgr8")

                # publish images through respective Publishers
                self.kinect_processed_thresh1.publish(ros_img1)
                self.kinect_processed_thresh2.publish(ros_img2)
                self.kinect_processed_final.publish(ros_img3)
                self.kinect_processed_pub.publish(ros_img4)
            except CvBridgeError as e:
                print(e)
                return
    
    # takes camera_info data, sets up camera_model as pinhole camera, then unregisters subscriber
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    # converts a point from RGB cam to Depth cam
    def coords_rgb2depth(self, coords):
        rc = [0,0]
        # removing the extra 210 pixels (total 420px from the width) 
        # and then scaling down from 1500px to 512px
        rc[0] = int((coords[0]-210)*0.34133)
        # sclaing down the height from 1080px to 374px and adding extra 30px
        rc[1] = int(coords[1]/1080.0 * 374) + 30
        if rc[0] < 0 or rc[1] < 0:
            return [-1,-1]
        return rc



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