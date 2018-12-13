#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib

from uol_cmp9767m_tutorial.msg import DoDishesAction, DoDishesResult

class DoDishesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
        self.server.start()
        self.total_dishes_cleaned = 0

    def execute(self, goal):
        # Do lots of robot stuff here
        # total dishes count is incremented each time the action is executed, and returned as result
        self.total_dishes_cleaned += 1
        result = DoDishesResult()
        result.total_dishes_cleaned = self.total_dishes_cleaned
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = DoDishesServer()
    rospy.spin()