#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Header
#
# Add code here to import the new ROS message...
#

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
      header = Header()
      header.stamp = rospy.Time.now()

      #
      # Add code here to create a new object of the new ROS message, to assign the random integers, and publish through the ROS topic...
      #
      
      rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass