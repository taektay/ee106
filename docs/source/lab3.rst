Lab 3: ROS Frames, Coordinate Systems, and the TF library
========================

Overview
--------

In this lab, we will investigate the reasons for using coordinate systems in nowadays robotic systems, and introduce the `ROS frames <http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF>`_ and the integrated `tf library <http://wiki.ros.org/tf>`_. Having that, we will demonstrate the existing coordinate systems on our Jackal robot inside the simulated world of Gazebo, we will create a new static transform and will try to spatially georeference each LiDAR measurement with respect to the newly created ROS frame. 

RViz and TF Visualisation
-----------

Initially, we start the Gazebo simulator with the simulated Jackal robot. Thus, in separate terminals execute,

.. code-block:: bash
 
 roslaunch gazebo_ros empty_world.launch

and 

.. code-block:: bash
 
 roslaunch ee106s24 jackal.launch

As the robot has been successfully spawned inside the Gazebo world, we can enable the ROS visualization tool, by executing in a separate terminal,

.. code-block:: bash
 
 rviz

In order to visualize the robot in the RViz tool, we have to set firstly the visualization `Global Options/Fixed Frame` to any of the listed coordinate systems. For our setup, we set `Global Options/Fixed Frame` to ``base_link``, as it represents the base coordinate system of the Jackal robot. Also, we by using the `Add` button of the left panel of RViz, we add the visualization of the `TF`, the `RobotModel`, and the `LaserScan` by selecting, the latter, to visualize the `front/scan` ROS topic. Finally, to include an obstacle inside the Gazebo world, we add a `Stop Sign` model at the position (2,0,0).

 .. image:: pics/jackal_stop_sign.png
 :align: center


 .. image:: pics/jackal_stop_sign_rviz.png
 :align: center


ROS Frames and TF Listener
-----------

`ROS frames <http://wiki.ros.org/tf2>`_ are fundamental entities in ROS, as they represent the existing coordinate systems of the robotic setup. Particularly, ROS frames can be assigned on any part of the robot, which can be considered rigid, as well as, on any onboard sensor. Thus, each captured measurement can be spatially described in the corresponding ROS frame of the capturing sensor, while multiple frames can be connected to each other spatially and form the ROS frame tree of the ROS setup.

 .. image:: pics/jackal_frames.jpg
 :align: center

In order to publish a transformation between two ROS frames that remains static over time, you can use the tool `static_transform_publisher` from the `tf` ROS package. For example in our case, it would be ideal to create a frame for the front bumper of the Jackal, so we can spatially describe all captured ranging measurements in respect to it to avoid any potential collisions as it moves forward.

To do that, we can describe the new `front_bumper` frame, with respect to the `base_link` frame of the robot, by executing in a new terminal, 

.. code-block:: bash

 rosrun tf static_transform_publisher 0.26 0 0.11 0 0 0 1 base_link front_bumper 100

where the arguments of this command are, 

.. code-block:: bash

 static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period(milliseconds)

One of the terminal commands to obtain the `transformation matrix` between two ROS frames is,

.. code-block:: bash

 rosrun tf tf_echo base_link front_laser

This information can be captured also inside a ROS node by using the ``tf.TransformListener()`` module. To test it, please initialize a new ROS node inside the ``ee106s24`` ROS package, under the name of ``tf_listener.py``, which will contain,

.. code-block:: python

 #!/usr/bin/env python
 import roslib
 roslib.load_manifest('ee106s24')
 import rospy
 import math
 import tf
 import geometry_msgs.msg
 import numpy as np

 # initialization of the ROS tf listener
 listener = tf.TransformListener()

 rate = rospy.Rate(10.0)
 # the goal of this node is to continously listen to the transformation relation between the base_link and front_laser ROS frames and print the Translation and Rotation of the captured transformation matrix.
 while not rospy.is_shutdown():
    try:
        # capture the tf of the two frames the exact moment of the command execution (rospy.Time(0))
        (trans,rot) = listener.lookupTransform('/base_link', '/front_laser', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation Matrix representation of the latter.
    print("The translation is (x,y,z) = " + str(trans))
    print("The rotation (quaternion) is (x,y,z,w) = " + str(rot))
    print("The rotation (euler) is (r,p,y) = " + str(tf.transformations.euler_from_quaternion(rot)))
    rot_mat = tf.transformations.quaternion_matrix(rot)
    print("The rotation (rotation matrix) is = " + str(tf.transformations.quaternion_matrix(rot)))
    
    # we assume that a Lidar point is detected, w.r.t the Lidar's frame
    laser_point_detected = [1, 0, 0, 1]
    
    # initialization of the tf matrix to describe it in the /base_link frame
    rot_mat[0,3] = trans[0]
    rot_mat[1,3] = trans[1]
    rot_mat[2,3] = trans[2]
    print(np.dot(rot_mat , laser_point_detected))
    
    rate.sleep()
 
 
Submission
-----------

#. Submission: individual submission via Gradescope

In this assignment, we will use our obstacle detection behavior (from Lab 2) based on LiDAR, to determine if an obstacle is close to the front_bumper. This behavior will be achieved by developing a new ROS node that will integrate a `tf listener`, along with a ROS subscriber and a publisher, to be able to receive the LiDAR measurements, transform them spatially, and determine the surrounding obstacle criticality. 

.. #. Demo: required (Demonstrate the ROS node functionality in the Gazebo world by using the Jackal.)

#. Due time: 11:59pm, May 4, Saturday

#. Files to submit: 

 - lab3_report.pdf (A template .pdf is provided for the report.) **Please include screenshots were possible and describe in detail all followed steps by showing the reasoning and any important remarks.** The developed Python code can be included in the end of your report, along with comments for describing the code parts.

#. Grading rubric:
 
 - \+ 10% Initialize the world setup as described above, by having the Jackal and the `Stop Sign` placed inside the Gazebo world. Also, create the new `front_bumper` frame, as described above.
 - \+ 10% Showcase how you can print the `transformation matrix` between the `front_laser` frame and the frame of the front bumper `front_bumper` by using the ``tf_echo`` command of the terminal. 
 - \+ 10% Create a new `ROS node <https://github.com/UCR-Robotics/ee106/blob/main/scripts/rangescheck_jackal.py>`_ that contains a ROS listener and obtain the transformation the `front_laser` and the `front_bumper` frames.
 - \+ 20% Print the translation and rotation matrices from the captured transformation and form the transformation matrix T [4x4].
 - \+ 10% Use the code of Lab 2 to subscribe to the `sensor_msgs/LaserScan` ROS topic of Jackal and obtain all the ranges that are not ``inf``. Use the integrated `calculate_position_of_range` method to obtain the positions of the captured ranges, with respect to the `front_bumper` frame. Explain the functionality of describing a capturing range of a LiDAR into a position. Why is it necessary? How does it work?
 - \+ 20% Transform all the ranged positions of the `front_laser` frame to the `front_bumper` frame, with the use of transformation matrix T.
 - \+ 20% Teleoperate the robot inside the world and print the transformed `non inf` ranges. Include a screenshot of the terminal including the robot, the laser scan, and the terminal output (print) of the ROS node.
 - \- 15% Penalty applies for each late day (up to two days). 
 


.. Solution Approach for Lab 3 Assignment
.. -----------------


.. .. code-block:: python

..     #!/usr/bin/env python3

..     import rospy
..     import roslib
..     roslib.load_manifest('ee106s24')
..     import sys
..     import numpy as np
..     from sensor_msgs.msg import LaserScan
..     import rospy
..     import math
..     import tf
..     import geometry_msgs.msg
..     import numpy as np

..     class ranges_check:
        
..     def __init__(self):
..         rospy.Subscriber("front/scan", LaserScan, self.callback)


..     def callback(self,data):

..         listener = tf.TransformListener()
        
..         while True:
..             try:
..                 (trans,rot) = listener.lookupTransform('/front_bumper', '/front_laser', rospy.Time(0))
..                 break
..             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
..                 continue
        
..         T = tf.transformations.quaternion_matrix(rot)
        
..         T[0,3] = trans[0]
..         T[1,3] = trans[1]
..         T[2,3] = trans[2]
..         print("\n\n")

..         idx = -1
..         for r in data.ranges:
..             idx = idx + 1
..             if (str(r)=="inf"):
..                 continue
            
..             (x,y) = self.calculate_position_of_range(r, idx, data.angle_increment, data.angle_min)
..             print(x,y)
..             laser_point_detected = [x, y, 0, 1]
..             print(np.dot(T , laser_point_detected))
            


..     def calculate_position_of_range(self, range, idx, angle_increment, angle_min):

..         if str(range)=="inf":
..             rospy.loginfo("The provided range is infinite!")
..             return -1

..         theta = idx * angle_increment + angle_min
..         x = range * np.cos(theta)
..         y = range * np.sin(theta)

..         return x,y

        
..     def main(args):
..         rospy.init_node('ranges_check', anonymous=True)
..         ic = ranges_check()
..         try:
..             rospy.spin()
..         except KeyboardInterrupt:
..             print("Shutting down")
            
..     if __name__ == '__main__':
..         main(sys.argv)



