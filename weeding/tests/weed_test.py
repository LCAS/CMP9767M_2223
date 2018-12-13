#!/usr/bin/env/python

import unittest

class WeedTest(unittest.TestCase):
    def test_two_two_two(self):
	self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    PKG = 'weeding'
    import rostest
    rostest.rosrun(PKG, 'WeedTest', WeedTest)
