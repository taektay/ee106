#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
#
# Add code here to import the new ROS message...
#


def callback(data):
    #
    # Add code here to perform the addition of the two integer field of the variable data
    #



def listener():
    rospy.init_node('listener')
    #
    # Initialize the ROS subscriber to capture the new  message-type ROS topic. The function "callback" will be the callback of the ROS subscriber.
    #
    rospy.spin()

if __name__ == '__main__':
    listener()