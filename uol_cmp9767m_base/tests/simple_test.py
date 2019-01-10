#!/usr/bin/env python

import unittest

#function taken from carrot_finder (stripped of self arguement as outside of class)
def getDisplacement(imageX, imageY):
        #function returns displacement of pixel from camera center in metres
        
        #whilst testing it was observed that when the camera is oriented directly down the following conditions are true
        #returning image is practically orthographic
        #takes a ground sample approximately 1m x 0.5m
        #for increased performance a simplified co-ordinate converter is used

        #camera parameters
        im_height = 1080
        im_width = 1920
        t_height = 0.5
        t_width = 0.9999999999

        pixel_height = t_height / im_height
        pixel_width = t_width / im_width

        #finding where the center of the image frame is (0,0)
        cent_left = 960 * pixel_width
        cent_top = 540 * pixel_height
        

        #finding how far from the left of the image a pixel is
        displacement_top = imageX * pixel_height
        displacement_left = imageY * pixel_width

        cam_displacement_x = cent_left - displacement_left
        cam_displacement_y = cent_top - displacement_top

        return cam_displacement_x, cam_displacement_y

# A sample python unit test
class CustomUnitTests(unittest.TestCase):
    def test_check_custom_tests_launch(self):
    	pass

    def test_getDisplacement(self):
    	#displacement from center of image should return 0,0
    	resultX, resultY = getDisplacement(540,960)

    	#if both results returned correctly then pass the test
    	if resultX == 0 and resultY == 0:
        	pass


if __name__ == '__main__':
    PKG = 'test_roslaunch'
    import rostest
    rostest.rosrun(PKG, 'test_unit_tests', CustomUnitTests)
