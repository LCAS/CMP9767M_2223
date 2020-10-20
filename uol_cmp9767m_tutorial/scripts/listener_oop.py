#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class ListenAndPublish():
    """
    A simple example of event-driven and OOP programming in ROS.
    When a new message is received on a topic, it also publishes
    data again on another topic. 
    """

    def __init__(self):
        """
        The "constructor" of the class, creating a Subscriber and registering 
        the callback, and also publisher to show how that could work
        """
        rospy.Subscriber("chatter", String, self.callback)
        self.pub = rospy.Publisher("count", Int32, queue_size=1)

        # here we initialise a member variable called count.
        # Note the "self" here, which corresponds to "this" in C++ and C#
        # This is the right way to store a state between callback calls
        self.count = 0

    def callback(self, data):
        """
        A simple callback that is called whenever there's new data available on the 
        topic. Simply prints it out.
        """
        # update the member variable (add 1)
        self.count += 1

        # the ROS way of logging, there's also rospy.logerr and rospy.logwarn for 
        # different logging levels
        rospy.loginfo(rospy.get_caller_id() + "I heard %s (message %d)", data.data, self.count)
        self.pub.publish(self.count)

if __name__ == '__main__':
    # ALWAYS first initialise your ros node with a name,
    # optional it can have added a unique identifier (anonymous) to all
    # starting the same code several times.
    rospy.init_node('listener', anonymous=True)

    # Instantiate the object
    l = ListenAndPublish()

    # "spin()" is a special method in ROS, which just keeps this
    # programm running and handling all incoming event (on subscribers)
    # to be processed. 
    rospy.spin()
