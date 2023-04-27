#!/usr/bin/env python3

import rospy
import sys
import numpy as np
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

  # calculation of the position of the range measurement with respect to the LiDAR frame
  def calculate_position_of_range(self, range, idx, angle_increment, angle_min):
    
    if str(range)=="inf":
          rospy.loginfo("The provided range is infinite!")
          return -1
          
    theta = idx * angle_increment + angle_min
    x = range * np.cos(theta)
    y = range * np.sin(theta)
    
    return x,y
    

    
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



