Lab 3: ROS Frames, Coordinate Systems, and the TF library
========================

Overview
--------

.. ADD

RViz and TF Visualisation
-----------

Initially, we start the Gazebo simulator with the simulated Jackal robot. Thus, in separate terminals execute,

.. code-block:: bash
    
    roslaunch gazebo_ros empty_world.launch

and 

.. code-block:: bash
    
    roslaunch ee106s23 jackal.launch

As the robot has been successfully spawned inside the Gazebo world, we can enable the ROS visualization tool, by executing in a separate terminal,

.. code-block:: bash
    
    rviz

In order to visualize the robot in the RViz tool, we have to set firstly the visualization `Global Options/Fixed Frame` to any of the listed coordinate systems. For our setup, we set `Global Options/Fixed Frame` to ``base_link``, as it represents the base coordinate system of the Jackal robot. Also, we by using the `Add` button of the left panel of RViz, we add the visualization of the `TF`, the `RobotModel`, and the `LaserScan` by selecting, the latter, to visualize the `front/scan` ROS topic. Finally, to include a obstacle inside the Gazebo world, we add a `Stop Sign` model at the position (2,0,0).

 .. image:: ./pics/jackal_stop_sign.png
    :align: center


 .. image:: ./pics/jackal_stop_sign_rviz.png
    :align: center


ROS Frames and TF Listener
-----------

`ROS frames <http://wiki.ros.org/tf2>`_ are fundamental entities in ROS, as they represent the existing coordinate systems of the robotic setup. Particularly, ROS frames can be assigned on any part of the robot, which can be considered rigid, as well as, on any onboard sensor. Thus, each captured measurement can be spatially described in the corresponding ROS frame of the capturing sensor, while multiple frames can be connected to each other spatially and form the ROS frame tree of the ROS setup.

 .. image:: ./pics/jackal_frames.jpg
    :align: center

In order to publish a transformation between two ROS frames that remains static over time, you can use the tool `static_transform_publisher` from the `tf` ROS package. For example in our case, it would be ideal to create a frame for the front bumper of the Jackal, so we can spatially describe all captured ranging measurents in respect to it to avoid any potential collisions as it moves forward.
To do that, we can describe the new `front_bumper` frame, with respect to the `base_link` frame of the robot, 

.. code-block:: bash

    rosrun tf static_transform_publisher base_link front_laser

One of the terminal commands to obtain the `transformation matrix` between two ROS frames is,

.. code-block:: bash

    rosrun tf tf_echo base_link front_laser

This information can be captured also inside a ROS node by using the ``tf.TransformListener()`` module. To test it, please initialize a new ROS node inside the ``ee106s23`` ROS package, under the name of ``tf_listener.py``, which will contain,

.. code-block:: python

    #!/usr/bin/env python  
    import roslib
    roslib.load_manifest('ee106s23')
    import rospy
    import math
    import tf
    import geometry_msgs.msg

    if __name__ == '__main__':
        rospy.init_node('tf_listener')

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
            print(rot_mat)

            rate.sleep()
        
        
.. Submission
.. -----------


.. #. Submission: individual submission via Gradescope

.. ADD

.. .. #. Demo: required (Demonstrate the ROS node functionality in the Gazebo world by using the Jackal.)

.. #. Due time: 11:59pm, May 7, Sunday

.. #. Files to submit: 

..    - lab3_report.pdf (A template .pdf is provided for the report.) **Please include screenshots were possible and describe in detail all followed steps by showing the reasoning and any important remarks.** The developed Python code can be included in the end of your report, along with comments for describing the code parts.

.. #. Grading rubric:
   
..    - \+ 10% Initialize the world setup as described above, by having the Jackal and the `Stop Sign` placed inside the Gazebo world.
..    - \+ 10% Showcase on how you can print the `transformation matrix` between the `front_laser` frame and the frame of the front bumper `` by using the ``tf_echo`` command of the terminal.   
..    - \+ 10% Create a new `ROS node <link>`_ that contains a ROS listener and obtain the transformation the `front_laser` and the `front_mount` frames.
..    - \+ 20% Print the translation and rotation matrices from the captured transformation and form the transformation matrix [4,x4].
..    - \+ 10% Use the code of Lab 2 to subscrive on the `sensor_msgs/LaserScan` ROS topic of Jackal and obtain the all the ranges that are not ``inf``.
..    - \+ 20% Transform all the ranges of the `front_laser` frame to the `front_mount` frame.
..    - \+ 20% Teleoperate the robot inside the world and print the transformed `non inf` ranges. Include a screenshot of the terminal including the robot, the laser scan, and the terminal output (print) of the ROS node.
..    - \- 15% Penalty applies for each late day (up to two days). 
  