#!/usr/bin/env python
import unittest, rospy, rostest, time
from std_msgs.msg import String

class TestMover(unittest.TestCase):
    def __init__(self, *args):
        super(TestMover, self).__init__(*args)
        self.success = False
    def callback(self, data):
        if "Plant Found" in data.data:
            self.success = True
    #image outputs a number
    def test_mover(self):
        rospy.init_node("Mover")
        rospy.Subscriber("/mover_state", String, self.callback)
        timeout_t = time.time() + 10
        while not rospy.is_shutdown()  and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))
        
if __name__ == '__main__':
    rostest.rosrun('Assessment', 'testimage', TestMover)
        
