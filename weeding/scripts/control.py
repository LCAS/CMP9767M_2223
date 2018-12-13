#!/usr/bin/env python
import rospy
import tf
import math

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from simple_move_base import Go_To_Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class weed_control():
    map_places = []
    spin_count = 0
    #move_base_commander = Go_To_Point()
    def __init__(self):
	#pubs
	self.spray_pub = rospy.Publisher('/spray', \
					 Bool, \
					 queue_size=10)
	self.find_weeds_pub = rospy.Publisher('/find_weeds', \
					      Bool, \
					      queue_size=10)
	self.go_pub = rospy.Publisher('/find_row', \
				      Bool, queue_size=10)
	self.search_pub = rospy.Publisher('/search_topic', \
					   Bool, \
					   queue_size=10)
	#subs
	self.spray_sub = rospy.Subscriber('/spray', \
					  Bool, \
					  self.spray_weeds_call)
	self.find_weeds_sub = rospy.Subscriber('/found_weeds', \
					       Bool, \
					       self.found_weeds_call)
	self.map_sub = rospy.Subscriber('/map', \
					OccupancyGrid,\
					self.create_map_places)
	self.odom_sub = rospy.Subscriber('/thorvald_001/odometry/gazebo', \
					 Odometry, \
					 self.odom_call)
	self.search_sub = rospy.Subscriber('/search_result', \
					   Bool, \
					   self.spinning)
	self.tf_listener = tf.TransformListener()
	self.unfin_path_sub = rospy.Subscriber('/unfinished_path', Bool, self.unfin_path_call)
	self.spray = False
	self.find_weeds = False
	self.ends = False
	self.odom = Odometry()
	self.search_found = False

    def odom_call(self, data):
	self.odom = data

    def create_map_places(self, data): #create searching points in the four corners of the square map
	y_trans = (data.info.height * data.info.resolution)/3
	x_trans = (data.info.width * data.info.resolution)/3
	self.map_places.append([x_trans, y_trans])
	self.map_places.append([x_trans*-1.0, y_trans])
	self.map_places.append([x_trans, y_trans*-1.0])
	self.map_places.append([x_trans*-1.0, y_trans*-1.0])
	self.map_sub.unregister()
	#self.search()

    def unfin_path_call(self, data):
	if data.data == True:#path not finished
		self.go_pub.publish(True)#keep doing vision runs
	elif data.data == False:#path finished
		if self.ends == False:#if no previous end
			move_base_commander = Go_To_Point()
			(roll, pitch, yaw) = euler_from_quaternion([self.odom.pose.pose.orientation.x, \
								   self.odom.pose.pose.orientation.y, \
								   self.odom.pose.pose.orientation.z, \
								   self.odom.pose.pose.orientation.w])

			yaw = yaw + math.pi
			if yaw >= (math.pi*2):
				yaw = yaw - (math.pi*2)
			quat = quaternion_from_euler (roll, pitch, yaw)
			go_place = []
			go_place.append(self.odom.pose.pose.position.x)
			go_place.append(self.odom.pose.pose.position.y)
			go_place.append(quat[0])
			go_place.append(quat[1])
			go_place.append(quat[2])
			go_place.append(quat[3])
			success = move_base_commander.point(go_place, 20)
			if success == True:
				self.go_pub.publish(True)#Do another vision run
				self.ends = True#change ends to True

		elif self.ends == True:#if previous end
			self.spray_pub.publish(True)#spray weeds[]

    def found_weeds_call(self, data):
	if data.data == True:#found weeds = True (happens is vision_test gets close to weeds)
		self.find_weeds_pub.publish(True)#start weed tracking		
	elif data.data == False:#row finished in vision_test
		self.find_weeds_pub.publish(False)#stop weed tracking
	
    def spray_weeds_call(self, data):
	if data.data == False:#spray finished
		self.search()#start search function

    def spinning(self, data):
	if data.data == False:
		self.spin_count += 1
		if self.spin_count == 6:
			self.spin_count = 0
			self.search()
		else:
			self.search_pub.publish(True)
			move_base_commander = Go_To_Point()
			(roll, pitch, yaw) = euler_from_quaternion([self.odom.pose.pose.orientation.x, \
								    self.odom.pose.pose.orientation.y, \
								    self.odom.pose.pose.orientation.z, \
								    self.odom.pose.pose.orientation.w])
			yaw1 = 0
			yaw2 = math.pi/2
			yaw3 = math.pi
			yaw4 = (math.pi/2)*3

			quat1 = quaternion_from_euler (roll, pitch, yaw1)
			quat2 = quaternion_from_euler (roll, pitch, yaw2)
			quat3 = quaternion_from_euler (roll, pitch, yaw3)
			quat4 = quaternion_from_euler (roll, pitch, yaw4)
			quats=[quat1,quat2,quat3,quat4,quat1]
			c = self.spin_count - 1
			go_place = []
			go_place.append(self.odom.pose.pose.position.x)
			go_place.append(self.odom.pose.pose.position.y)
			go_place.append(quats[c][0])
			go_place.append(quats[c][1])
			go_place.append(quats[c][2])
			go_place.append(quats[c][3])
			success = move_base_commander.point(go_place, 20)
	elif data.data == True:
		self.go_pub.publish(True)

    def search(self):
	if len(self.map_places) == 0:
		return
	self.ends = False#change ends to False
	x = self.map_places.pop()
	go_place = []
	go_place.append(x[0])
	go_place.append(x[1])
	go_place.append(0)
	go_place.append(0)
	go_place.append(0)
	go_place.append(1)
	move_base_commander = Go_To_Point()
	success = move_base_commander.point(go_place, 120) #go_place = location for sprayer, 120 = time(s) until goal is rejected
	self.search_pub.publish(True)
			
rospy.init_node('weed_control', anonymous=True)
wc = weed_control()
rospy.spin()

