#!/usr/bin/env python3

import rospy
import sys
#
# Add code here to import the sensor_msgs/LaserScan and std_msgs/String message...
#


class ranges_check:
    
  def __init__(self):
    #
    # Initialize the ROS publisher and subscriber. Use "self." to initialize the publisher and subscriber variables, to be able to access them through all class methods. The function "callback" will be the callback of the ROS subscriber. 
    #

  def callback(self,data):

    # Add code here to iterate over all values in LaserScan ranges[] field and check the criticality of the robot position. Additionally, initialize a String variable that will contain the criticality message.
    #
    
    # for i in ...

    # Publish the String through the created ROS publisher variable...
    #


    
def main(args):
    ## initialization of the class object
    ic = ranges_check()
    rospy.init_node('ranges_check', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)



