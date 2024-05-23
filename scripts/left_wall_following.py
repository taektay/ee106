#!/usr/bin/env python3

import roslib
roslib.load_manifest('ee106s24')
import rospy
import sys
import tf
import numpy as np
from math import pi, sqrt
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

class Turtlebot():
    def __init__(self, goal_x, goal_y, csv_file):
        # Data to be taken from Launch File. Dont Edit
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.csv_file = csv_file

        # Initialize subscribers and publishers
        # Edit Code Here

        # Write a LiDAR Subscriber
        # Lidar Sub to be connected to self.lidar_callback
        self.lidar_sub = 

        # Write code to publish velocity to Bot
        self.vel_pub = 

        # Write an Odometry Subscriber
        # Lidar Sub to be connected to self.odom_callback
        self.odom_sub = 

        self.rate = rospy.Rate(10)

        # Initialize state variables
        self.left = "occupied"
        self.front = "free"
        self.wall = "fine"
        self.state = "forward"
        self.straight = "left"
        self.direction = 0
        self.logging_counter = 0
        self.left_min_dist = 9999
        self.forward_dist = 9999
        self.pose = Pose2D()
        self.tolerance = 0.3
        self.trajectory = []

        # Define state transition matrix using NumPy
        self.state_transition_matrix = np.array([
            #left Side: Free
            [    
                # Front side: free  -> Left
                [1],                
                [2] 
            ],
            #left Side: Occupied
            [    
                [0],
                [2]
            ]
        ])

        # Define state and condition encoding
        self.state_encoding = {'forward': 0, 'left': 1, 'right': 2}
        self.state_decoding = {0: 'forward', 1: 'left', 2: 'right'}
        self.condition_encoding = {'free': 0, 'occupied': 1, 'fine': 0, 'close': 1}

        # Start the main loop
        self.run()
    
    def run(self):
        # Dont Edit anything
        # Main loop that runs until the node is shutdown
        while not rospy.is_shutdown():
            # Finite-State Machine
            self.update_state() # Update the robot's state based on sensor readings
            self.publish_velocity() # Publish velocity commands based on the current state
            self.rate.sleep()  # Sleep to maintain the loop rate
    
    def update_state(self):
        # State machine to update the robot's state based on sensor readings
        current_state_encoded = self.state_encoding[self.state]
        wall_condition_encoded = self.condition_encoding[self.wall]
        left_condition_encoded = self.condition_encoding[self.left]
        front_condition_encoded = self.condition_encoding[self.front]

        # Write code to encode current state and conditions
        # Get the new state from the state transition matrix



        # Decode the new state



    
    def publish_velocity(self):
        
        # Define named constants for directions
        SLIGHT_RIGHT = 1
        SLIGHT_LEFT = 2
        SHARP_RIGHT = 3
        SHARP_LEFT = 4
        MODERATE_RIGHT = 5
        MODERATE_LEFT = 6

        # Publish velocity commands based on the current state
        # Fill in the velocities, keep values minimal
        # Keep editing values in the given range till the robot moves well.

        # Velocity values are good in the range of (0.07 to 0.12)
        # Angular Velocities are good in the range of (-0.07 to 0.07)
        # Also update values for self.direction
        vel = Twist()
        if self.state == "forward":
            if self.straight == "forward":
                if self.direction == :
                    vel.linear.x = 
                    vel.angular.z = 
                elif self.direction == :
                    vel.linear.x = 
                    vel.angular.z = 
                elif self.direction == :
                    vel.linear.x = 
                    vel.angular.z = 
                else:
                    vel.linear.x = 
                    vel.angular.z = 
            elif self.straight == "left":
                vel.linear.x = 
                vel.angular.z = 
                self.direction = 1
            elif self.straight == "right":
                vel.linear.x = 
                vel.angular.z = 
                self.direction = 
        elif self.state == "left":
            vel.linear.x = 
            vel.angular.z = 
        elif self.state == "right":
            if self.direction == :
                vel.linear.x = 
                vel.angular.z = 
            elif self.direction == :
                vel.linear.x = 
                vel.angular.z = 
            else:
                vel.linear.x = 
                vel.angular.z = 

        self.vel_pub.publish(vel)

    def lidar_callback(self, data):
        # Callback function to handle incoming LIDAR data
        self.transformed_ranges = []
        self.left_min_dist = 100

        # Update the forward distance with the distance directly in front of the robot
        self.forward_dist = 9999 if str(data.ranges[0]) == "inf" else data.ranges[0]

        # Write transformation listener code to get the position of the LIDAR relative to the robot
        # transform between /rplidar_link and /cliff_sensor_left_link
        listener = tf.TransformListener()
        while True:
            try:
                (trans, rot) = 
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        # Process the LIDAR data and transform the points to the robot's coordinate frame
        for i in range(len(data.ranges)):
            if 1.22 < i * data.angle_increment < 1.92:
                dist = 9999 if str(data.ranges[i]) == "inf" else data.ranges[i]
                (x, y) = self.calculate_position_of_range(dist, i, data.angle_increment, data.angle_min)
                # Write code below 



                self.transformed_ranges.append(transformed_point)
        
    
        # Determine the minimum distance to the left of the robot
        for point in self.transformed_ranges:
            # Point is a list of transformed LiDAR Readings
            # Write code to determine the minimum distance to an obstacle on the left side of the robot. 
            # Work with dist and self.left_min_dist. The final output, should be returned in self.left_min_dist

        



        # Update the state variables based on the distances
        self.left = "occupied" if self.left_min_dist < 0.25 else "free"
        self.front = "occupied" if self.forward_dist < 0.5 else "free"
        self.wall = "close" if self.left_min_dist < 0.1 else "fine"

        # Update the direction the robot should go based on the distance to the left
        if self.left_min_dist < 0.15:
            self.straight = "right"
        elif self.left_min_dist > 0.17:
            self.straight = "left"
        else:
            self.straight = "forward"
        
        # Log the state information
        rospy.loginfo(f"{self.left} {self.front} {self.wall} {self.left_min_dist} {self.forward_dist} {self.state}")

    def calculate_position_of_range(self, dist, idx, angle_increment, angle_min):
        # Calculate the (x, y) position of a LIDAR range reading in the robot's coordinate frame
        if str(dist) == "inf":
            rospy.loginfo("The provided range is infinite!")
            return -1
            
        # Write code to find values of theta, x & y in Robot's frame of reference
        # Lidar is in Polar Co-ordinates
        # Robot is in Cartesia Co-ordinates




        return x, y
        
    def save_trajectory(self):
        # Save the trajectory to a CSV file, csv file name is given in launch file. Nothing to edit here
        np.savetxt(self.csv_file, np.array(self.trajectory), fmt='%f', delimiter=',')
        
    def odom_callback(self, msg):
        # Callback function to handle incoming odometry data
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Write code heck if the robot has reached the goal within the tolerance
        # if robot has reached goal, save the trajectory by self.save_trajectory(), and then shutdown ROS using rospy.signal_shutdown()
        



        # Log the odometry data every 100 iterations
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])
            rospy.loginfo(f"odom: x={self.pose.x}; y={self.pose.y}")


def main(args):
    # Initialize ROS Node
    rospy.init_node('left_wall_follower', anonymous=True)

    # Get parameters from the launch file
    goal_x = rospy.get_param('goal_x', 8)
    goal_y = rospy.get_param('goal_y', 5)
    csv_file = rospy.get_param('csv_file', 'trajectory.csv')

    # Create an instance of the Turtlebot class
    Turtlebot(goal_x, goal_y, csv_file)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
