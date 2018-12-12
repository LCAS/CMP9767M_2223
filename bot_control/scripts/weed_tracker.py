#!/usr/bin/env python

# Python libs
import sys
import uuid

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from geometry_msgs.msg import PointStamped
from bot_control.msg import WeedList, UniqueWeed
from std_srvs.srv import Empty

class WeedTracker:
    def __init__(self):
        self.weed_list = [] # local list of unique weeds
        self.match_tolerance_radius = 0.008 # square of max distance between two points to be considered the same weed

        # service to call when a weed has been sprayed/skipped
        rospy.Service('/bot_control/weed_passed', Empty, self.weed_passed)

        # Publishes a list of weeds of type WeedList
        self.weed_list_pub = rospy.Publisher('/thorvald_001/weed_list', 
            WeedList, queue_size=1)

        # Subscriber for weed points detected in kinect_processor.py
        rospy.Subscriber("/thorvald_001/weed_location",
            PointStamped, self.weed_loc_callback)
        
        # transform listener
        self.tf_listener = tf.TransformListener()

    # removes first weed from weed_list
    def weed_passed(self, r):
        # print("deleting")
        if len(self.weed_list) > 0:     # check if weed_list is not empty to prevent exception in pop(i)
            self.weed_list.pop(0)
            # print("Popped")
        return []
    
    # callback for /thorvald_001/weed_location subscriber
    def weed_loc_callback(self, data):
        self.find_weed(data)
        self.weed_list_pub.publish(self.weed_list)

    # looks for weed in weed_list. Append to it if not found.
    def find_weed(self, data):
        sort_index = 0
        found_index = False
        for i, weed in enumerate(self.weed_list):
            if ((weed.point.point.x - data.point.x)**2 + (weed.point.point.x - data.point.x)**2) < self.match_tolerance_radius:
                self.weed_list[i].point.point.x = (weed.point.point.x*weed.confidence + data.point.x)/(weed.confidence+1)
                self.weed_list[i].point.point.y = (weed.point.point.y*weed.confidence + data.point.y)/(weed.confidence+1)
                self.weed_list[i].confidence += 1
                return
            if found_index:
                continue
            # weed.point.header.stamp = rospy.Time(0)
            try:
                curr_weed_point = self.tf_listener.transformPoint('thorvald_001/base_link', weed.point)
                new_weed_point = self.tf_listener.transformPoint('thorvald_001/base_link', data)
                if new_weed_point.point.x > curr_weed_point.point.x:
                    sort_index += 1
                else:
                    found_index = True
            except tf.ExtrapolationException:
                # print(weed)
                return

        weed = UniqueWeed()
        weed.uuid = bytes(uuid.uuid4())
        weed.confidence = 1
        weed.point = data
        self.weed_list.insert(sort_index, weed)
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