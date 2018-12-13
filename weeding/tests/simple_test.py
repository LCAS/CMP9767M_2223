#!/usr/bin/env python

import unittest


# A sample python unit test
class TestBareBones(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!
    def setUp(self):
	self.x = 5

    def test_two_equals_two(self):
	if self.x == 5:
		self.assertTrue(True)
        else:
		self.assertTrue(False, msg="5 not 5")

if __name__ == '__main__':
    #PKG = 'test_roslaunch'
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
