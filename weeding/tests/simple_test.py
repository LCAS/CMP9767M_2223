#!/usr/bin/env python

import unittest


# A sample python unit test
class TestBareBones(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!
    def test_two_equals_two(self):
        self.assertEquals(2, 2, "2!=2")

if __name__ == '__main__':
    #PKG = 'test_roslaunch'
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
