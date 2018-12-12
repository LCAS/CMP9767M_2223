#!/usr/bin/env python

import rospy
import actionlib
from cv2 import namedWindow, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import waitKey, morphologyEx, MORPH_CLOSE, MORPH_OPEN
from cv2 import threshold, THRESH_BINARY, split, medianBlur, connectedComponents
from cv2 import merge, cvtColor, COLOR_HSV2BGR
from numpy import ones, uint8, max, ones_like
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        #initialising subscribers
        #initialising subscriber for camera data
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
                                          Image, self.img_callback)
        #subscriber for fake localisation odometry data
        self.odom_sub = rospy.Subscriber("/thorvald_001/odometry/base_raw", Odometry, self.odom_callback)

        #subscriber for testing flag

        #initialising kernels for opening and closing operations                                   
        self.kernel = ones((9,9), uint8)     
        self.kernel_small = ones((5,5), uint8)

        #creating the move base client
        self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)          #POTENTIALLY PUBLISHING TO WRONG TOPIC, CHECK THIS
        self.move_client.wait_for_server(rospy.Duration(5))

        

        #initialising local odometry flags
        self.posX = 0               #robot X pos
        self.posY = 0               #robot Y pos
        self.posZ = 0               #robot Z pos
        self.angularX = 0           #robot angularX
        self.angularY = 0           #robot angularY
        self.angularZ = 0           #robot angularZ        

    def img_callback(self, data):
        #OPENCV DISPLAY REMOVED FOR INCREASED PERFORMANCE
        #namedWindow("Image window")

        #defining publisher for output image
        camera_objects = rospy.Publisher("camera_objects", Image, queue_size = 1)  

        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #splitting image into b,g,r channels        
        b,g,r = split(cv_img)        
        #performing binary thresholding operation on green channel
        ret, thresh = threshold(g, 50, 255, THRESH_BINARY)
        
        #opening image to remove small objects/noise 
        open_result = morphologyEx(thresh, MORPH_OPEN, self.kernel)
        
        #performing closing operation to remove voids in remaining objects
        closed_img = morphologyEx(open_result, MORPH_CLOSE, self.kernel_small)
        
        #finding the connected components from the closed image, Connected Components Example adapted from A.Reynolds (2017) Source: https://stackoverflow.com/questions/46441893/connected-component-labeling-in-python?rq=1
        retval, labels = connectedComponents(closed_img)        
        
        #iterating through the labels to find carrot/non-carrot objects
        
        # Map component labels to hue val
        label_hue = uint8(179*labels/max(labels))
        blank_ch = 255*ones_like(label_hue)
        labeled_img = merge([label_hue, blank_ch, blank_ch])

        # cvt to BGR for display
        labeled_img = cvtColor(labeled_img, COLOR_HSV2BGR)

        # set bg label to black
        labeled_img[label_hue==0] = 0
        

        #displaying the result of thresholding, and closing operations
        #OPENCV DISPLAY REMOVED FOR INCREASED PERFORMANCE#imshow("Image window", labeled_img)
        ##END DISPLAY METHODS        
        
        #publishing image data to camera_objects topic
        output = self.bridge.cv2_to_imgmsg(labeled_img, encoding="rgb8")
        camera_objects.publish(output)

        waitKey(1)

    def odom_callback(self, data):
        #callback method for the odometry subscriber
        #updates local flags with current odometry
        self.posX = data.pose.pose.position.x                  #robot X pos
        self.posY = data.pose.pose.position.y                  #robot Y pos
        self.posZ = data.pose.pose.position.z                  #robot Z pos
        self.angularX = data.pose.pose.orientation.x           #robot angularX
        self.angularY = data.pose.pose.orientation.y           #robot angularY
        self.angularZ = data.pose.pose.orientation.z           #robot angularZ       

        self.navigateTo(1,1,1)
        #uncomment for output of odometry
        #self.print_local_odometry()

    def print_local_odometry(self):
        #outputs to terminal the current values in the local odometry    
        print "pos x: " + str(self.posX) 
        print "pos y: " + str(self.posY) 
        print "pos z: " + str(self.posZ) 
        print "orientation x: " + str(self.angularX) 
        print "orientation y: " + str(self.angularY) 
        print "orientation z: " + str(self.angularZ)  

    def flag_callback(self, data):
        #callback method for flag test
        print "test"

    #def getCoOrds(self, imageX, imageY):
        #function returns spatial co-ords relative to robots frame given a camera value
        #camera parameters
    #    camera_angle = 45           #angle of camera
    #    cam_height = 0.5            #height of camera from the ground
    #    cam_displacement_x = 0.0    #displacement of the camera from robot center on x axis
    #    cam_displacement_y = 0.0    #displacement of the camera from robot center on y axis

    def navigateTo(self, worldX, worldY, orientation):
        #function sets a goal for the robot to navigate to 

        #creating a new MoveBaseGoal, and target pose
        goal = MoveBaseGoal() 
        goal_pose = Pose() 

        #setting desired pose 
        goal_pose.position.x = worldX 
        goal_pose.position.y = worldY 
        goal_pose.position.z = 0 
        goal_pose.orientation.w = orientation # 1 for parallel to rows
        goal_pose.orientation.x = 0 
        goal_pose.orientation.y = 0 
        goal_pose.orientation.z = 0 

        goal.target_pose.header.stamp = rospy.Time.now() # set MoveBaseGoal time stamp to current time from rospy
        goal.target_pose.header.frame_id = 'map' # set the frame to map
        goal.target_pose.pose = goal_pose # set the pose to the above
       
        #sending goal to actionlib server
        self.move_client.send_goal(goal) 
        
        print "Goal Set"

    def cancelGoal(self):
        #function publishes an empty nav goal to clear the queue (as a side effect stops the robot)
        print "test"

    def sprayGround(self, worldX, worldY):
        #function handles the sprayer mechanism
        print "test"    

startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()