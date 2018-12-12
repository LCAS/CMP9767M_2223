#!/usr/bin/env python

import unittest


# A sample python unit test
class Unit_Test(unittest.TestCase):
    # test 1 == 1
    # only functions with 'test_'-prefix will be run!
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    PKG = 'test_roslaunch'
    import rostest
    rostest.rosrun(PKG, 'unit_test', Unit_Test)
