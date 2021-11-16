#!/usr/bin/env python

# Python libs
import sys, time

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class image_projection:
    camera_model = None

    def __init__(self):    

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_front_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
            Image, self.image_callback)

    def image_callback(self, data):
        if not self.camera_model:
            return

        #project a point in camera coordinates into the pixel coordinates
        uv = self.camera_model.project3dToPixel((0,0,1.0))

        print 'Pixel coordinates: ', uv
        print ''

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.circle(cv_image, (int(uv[0]),int(uv[1])), 10, 255, -1)

        #resize for visualisation
        cv_image_s = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)

        cv2.imshow("Image window", cv_image_s)
        cv2.waitKey(1)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
