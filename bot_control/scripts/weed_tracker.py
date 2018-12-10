#!/usr/bin/env python

# Python libs
import sys
import uuid

# OpenCV
import cv2

# Ros libraries
import roslib, rospy

# Ros Messages
from geometry_msgs.msg import PointStamped
from bot_control.msg import WeedList, UniqueWeed

class WeedTracker:
    def __init__(self):    
        self.weed_list = []
        self.match_tolerance_radius = 0.0016

        # Weed 3d point: Published the weed's coordinates in the map frame
        self.weed_list_pub = rospy.Publisher('/thorvald_001/weed_list', 
            WeedList, queue_size=1)

        # Subscribers for kinect
        # Kinect rgb sensor data is 420 pixels wider than depth sensor data
        rospy.Subscriber("/thorvald_001/weed_location",
            PointStamped, self.weed_loc_callback)

    def weed_loc_callback(self, data):
        self.find_weed(data)
        self.weed_list_pub.publish(self.weed_list)

    def find_weed(self, data):
        for i, weed in enumerate(self.weed_list):
            if ((weed.point.point.x - data.point.x)**2 + (weed.point.point.x - data.point.x)**2) < self.match_tolerance_radius:
                self.weed_list[i].point.point.x = (weed.point.point.x + data.point.x)/2
                self.weed_list[i].point.point.y = (weed.point.point.y + data.point.y)/2
                self.weed_list[i].confidence += 1
                return
        weed = UniqueWeed()
        weed.uuid = bytes(uuid.uuid4())
        weed.confidence = 1
        weed.point = data
        self.weed_list.append(weed)
        print(len(self.weed_list))

def main(args):
    '''Kinect Processor node'''
    rospy.init_node('weed_tracker', anonymous=True)
    wt = WeedTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)