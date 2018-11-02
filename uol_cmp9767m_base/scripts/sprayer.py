#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from std_srvs.srv import Empty
from uuid import uuid4

BOX_SDF="""
<?xml version='1.0'?>
<sdf version="1.4">
<model name="killbox">
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.00083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.00083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.0000083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>.1 .1 .01</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>.1 .1 .01</size>
          </box>
        </geometry>
        <material>
            <ambient>1 0 0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class Sprayer:

    def __init__(self):
        self.sdf = BOX_SDF
        rospy.Service('spray', Empty, self.spray)
        self.spawner = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel)

    def spray(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []


if __name__ == "__main__":
    rospy.init_node('sprayer')
    m2s = Sprayer()
    #m2s.spray()
    #m2s.load_file('test.yaml')
    #m2s._create_svg()
    #m2s.write_svg()
    rospy.spin()
