#!/usr/bin/env python3

import roslib
roslib.load_manifest('ee106s24')
import rospy
import sys
import csv
import tf
import numpy as np
from math import pi, sqrt
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import os

class PDController:
    def __init__(self, P=1.0, D=0.1, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        #  e = r-y
        error = self.set_point - current_value 
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):  
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot():
    def __init__(self, goal_x, goal_y, csv_file):
        # Data to be taken from Launch File. Dont Edit
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.csv_file = csv_file
        self.complex_trajectory_file = "complex_trajectory.csv"
        
        # Initialize subscribers and publishers
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        
        # Initialize state variables
        self.left = "O"
        self.front = "F"
        self.wall = "S"
        self.state = "forward"
        self.current_facing = 0  # straight
        self.logging_counter = 0
        self.left_min_dist = 100
        self.forward_min_dist = 999
        self.pose = Pose2D()
        self.goal_tolerance = 0.3
        self.trajectory = list()
        self.angular_threshold = 0.003
        self.pd_control = PDController()
        self.pd_control.setPD(1.0, 0.1)

        # For the complex world map
        self.control_list = [pi/2, pi]

        # Define Finite-State Machine matrix by NumPy
        self.state_transition_matrix = np.array([
            # Left Side: Free
            [   # Front side: free  -> Forward
                0,        
                # Front side: Occupied  -> Right
                2 
            ],
            # Left Side: Occupied
            [   # Front side: free  -> Left
                1,
                # Front side: Occupied  -> Right
                2
            ]
        ])

        # Define state and condition encoding
        self.state_encoding = {'forward': 0, 'left': 1, 'right': 2}
        self.state_decoding = {0: 'forward', 1: 'left', 2: 'right'}
        self.condition_encoding = {'F': 0, 'O': 1}  # F: free; O: Occupied
        self.current_facing_decoding = {0: "straight", -1: "small_left", -2: "medium_left", -3: "large_left",
                                        1: "small_right", 2: "medium_right", 3: "large_right"}
        self.run()


    # IMPORT CSV FILE
    def load_trajectory(self, csv_file):
        self.trajectory = []
        try:
            with open(csv_file, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    self.trajectory.append((float(row['time']), float(row['linear_x']), float(row['angular_z'])))
        except Exception as e:
            rospy.logerr(f"Error reading CSV file: {e}")
            self.trajectory = []
    
    def run(self):
        # Don't edit anything
        while not rospy.is_shutdown():
            self.update_state()  # Update the robot's state based on sensor readings
            # Publish velocity commands based on the current state
            self.publish_velocity() 
            # Sleep to maintain the loop rate
            self.rate.sleep()  

    def update_state(self):
        # State machine to update the robot's state based on sensor readings
        current_state_encoding = self.state_encoding[self.state]
        left_cond_encoding = self.condition_encoding[self.left]
        front_cond_encoding = self.condition_encoding[self.front]

        # Write code to encode current state and conditions
        # Get the new state from the state transition matrix
        new_state_encoded = self.state_transition_matrix[left_cond_encoding, front_cond_encoding]

        # Decode the new state
        self.state = self.state_decoding[new_state_encoded]

    def publish_velocity(self):
        vel = Twist()
        # Publish velocity commands based on the current facing direction
        # Fill in the velocities, keep the values small
        # Keep editing values in the given range till the robot moves well.

        # Velocity values are good in the range of (0.01 to 0.2)
        # Angular Velocities are good in the range of (-0.08 to 0.08)
        if self.current_facing_decoding[self.current_facing] == "straight":  # 0
            vel.linear.x = 0.1
            vel.angular.z = 0.0
        if self.current_facing_decoding[self.current_facing] == "small_left":  # -1
            vel.linear.x = 0.1
            vel.angular.z = 0.03
        if self.current_facing_decoding[self.current_facing] == "medium_left":  # -2
            vel.linear.x = 0.1
            vel.angular.z = 0.05
        if self.current_facing_decoding[self.current_facing] == "small_right":  # 1
            vel.linear.x = 0.1
            vel.angular.z = -0.03
        if self.current_facing_decoding[self.current_facing] == "medium_right":  # 2
            vel.linear.x = 0.1
            vel.angular.z = -0.05

        print(self.left + ' ' + self.front + ' ' + self.wall + ' ' 
              + str(round(self.left_min_dist, 3)) + ' ' + str(round(self.forward_min_dist, 3)) 
              + ' ' + self.state + ' ' + str(self.current_facing))
        
        if self.state == "left":
            if len(self.control_list) < 1:
                rospy.signal_shutdown('Received shutdown message')

            # Move forward a bit: from lab 4
            for i in range(10):
                vel.linear.x = 0.1
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                self.rate.sleep()

            # Use PD controller to control the angle: from Lab 5
            while abs(self.pose.theta - self.control_list[0]) > self.angular_threshold:
                vel.angular.z = self.pd_control.update(self.pose.theta - self.control_list[0])
                vel.linear.x = 0.0
                self.vel_pub.publish(vel)
                self.rate.sleep()
            self.control_list.pop(0)
            print("done with turning")

            # Move forward a bit: from lab 4
            for i in range(10):
                vel.linear.x = 0.1
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                self.rate.sleep()
        else:
            self.vel_pub.publish(vel)

    def lidar_callback(self, data):
        self.left_min_dist = 100
        
        # Update the forward distance with the distance directly in front of the robot
        if str(data.ranges[0]) == "inf":
            self.forward_min_dist = 999
        else:
            self.forward_min_dist = data.ranges[0]

        # Transform the lidar points frame /rplidar_link from to another frame: from lab 3 and lab 2
        listener = tf.TransformListener()
        listener.waitForTransform('/base_link', '/laser', rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/base_link', '/laser', rospy.Time(0))

        # Process the LIDAR data and transform the points to the robot's coordinate frame (another frame you specified)
        for i in range(len(data.ranges)):
            # Get the left side lidar data
            if i * data.angle_increment < 1.59 and i * data.angle_increment > 1.55:
                if str(data.ranges[i]) == "inf":
                    dist = 9999
                else:
                    dist = data.ranges[i]   
                (x, y) = self.calculate_position_of_range(dist, i, data.angle_increment, data.angle_min)
                point = Pose2D(x, y, 0)
                listener.waitForTransform('/base_link', '/laser', rospy.Time(0), rospy.Duration(4.0))
                transformed_point = listener.transformPose('/base_link', point)

                left_dist = transformed_point.x
                self.left_min_dist = min(self.left_min_dist, left_dist)

        # Update left and forward state
        if self.left_min_dist < 0.3:
            self.left = "O"
        else:
            self.left = "F"
        if self.forward_min_dist < 0.3:
            self.front = "O"
        else:
            self.front = "F"

        # Set wall state
        if self.left_min_dist < 0.1:
            self.wall = "C"
        else:
            self.wall = "S"

        # Update current_facing direction
        if self.left_min_dist < 0.15:
            self.current_facing = 2  # medium_right
        elif self.left_min_dist < 0.25:
            self.current_facing = 1  # small_right
        elif self.left_min_dist > 0.35 and self.left_min_dist < 0.45:
            self.current_facing = -1  # small_left
        elif self.left_min_dist >= 0.45:
            self.current_facing = -2  # medium_left
        else:
            self.current_facing = 0  # straight

    def calculate_position_of_range(self, range, idx, angle_increment, angle_min):
        if str(range) == "inf":
            rospy.loginfo("The provided range is infinite!")
            return -1, -1
        theta = idx * angle_increment + angle_min
        x = range * np.cos(theta)
        y = range * np.sin(theta)
        return x, y
    
    def save_trajectory(self):
        # Save the trajectory to a CSV file
        np.savetxt(self.csv_file, np.array(self.trajectory), fmt='%f', delimiter=',')
        np.savetxt(self.complex_trajectory_file, np.array(self.trajectory), fmt='%f', delimiter=',')

    def odom_callback(self, msg):
        # Callback function to handle incoming odometry data
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Check if the robot has reached the goal within the tolerance
        distance_to_goal = sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
        if distance_to_goal < self.goal_tolerance:
            self.save_trajectory()
            rospy.signal_shutdown("Goal reached")

        # Log the odometry data every 100 iterations
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) 

def main(args):
    # Initialize ROS Node
    rospy.init_node('left_wall_follower', anonymous=True)
    # Get parameters from the launch file
    goal_x = rospy.get_param('goal_x')
    goal_y = rospy.get_param('goal_y')
    csv_file = rospy.get_param('csv_file')

    # Create an instance of the Turtlebot class
    robot = Turtlebot(goal_x, goal_y, csv_file)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
