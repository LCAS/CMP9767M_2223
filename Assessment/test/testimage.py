#!/usr/bin/env python
import unittest, rospy, rostest, time
from std_msgs.msg import Float32

class TestImageProcessing(unittest.TestCase):
    def callback(self, data):
        self.success = data.data >= 0.0 and isinstance(data.data, float)
    #image outputs a number
    def test_image_processing(self):
        rospy.init_node("image_processing")
        rospy.Subscriber("/colorval_camera", Float32, self.callback)
        timeout_t = time.time() + 10
        while not rospy.is_shutdown()  and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.rosrun('Assessment', 'testimage', TestImageProcessing)
        
