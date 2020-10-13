#!/usr/bin/env python

import psutil
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('freemem', String, queue_size=10)
    rospy.init_node('ProprioceptiveSensing', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = 
        info = psutil.virtual_memory()
        available_mem = info.available
        #print available_mem
        pub.publish(str(available_mem))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
