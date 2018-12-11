#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from simple_move_base import Go_To_Point

class weed_control():
	
    def __init__(self):

	self.spray_pub = rospy.Publisher('/spray', \
					 Bool, \
					 queue_size=10)

	self.find_weeds_pub = rospy.Publisher('/find_weeds', \
					      Bool, \
					      queue_size=10)

	self.spray_sub = rospy.Subscriber('/spray', \
					  Bool, \
					  self.spray_weeds_call)

	self.map_sub = rospy.Subscriber('/map', \
					OccupancyGrid,\
					self.create_map_places)
	self.spray = False
	self.find_weeds = False
	self.map_places = []

    def create_map_places(self, data): #create searching points in the four corners of the square map
	y_trans = (data.info.height * data.info.resolution)/3
	x_trans = (data.info.width * data.info.resolution)/3
	self.map_places.append([x_trans, y_trans])
	self.map_places.append([x_trans*-1.0, y_trans])
	self.map_places.append([x_trans, y_trans*-1.0])
	self.map_places.append([x_trans*-1.0, y_trans*-1.0])
	self.map_sub.unregister()
	self.search()
	
    def spray_weeds_call(self, data):
	if data.data == False:
		self.spray = False #spraying finished
		self.search()

    def search(self):
	if len(self.map_places) == 0:
		return
	move_base_commander = Go_To_Point()
	x = self.map_places.pop()
	go_place = []
	go_place.append(x[0])
	go_place.append(x[1])
	go_place.append(0)
	go_place.append(0)
	go_place.append(0)
	success = move_base_commander.point(go_place, 120) #go_place = location for sprayer, 120 = time(s) until goal is rejected
	if success == True:
		pass 
	
rospy.init_node('weed_control')
wc = weed_control()
rospy.spin()

