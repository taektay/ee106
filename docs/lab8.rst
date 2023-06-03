Lab 8: Go! Turtlebot!
=====================

Overview
--------

In this lab, we will put everything together and apply what we have learned so far in ROS on the real Turtlebot3 Burger. 
The task is to use Turtlebot3 and perform left-wall following in a real-world environment without colliding with obstacles or walls and finally kicking the ball at the endpoint of the map.

The implemented ROS node of Lab 7 will be used with proper modifications, to run onboard the Turtlebot3 to complete the navigation scenario. In the end, a demonstration of the complete wall-following behavior will be performed for the system's evaluation. Each team will have one Turtlebot3 to work with and do the demonstration.

**A successful demo on Gazebo is required before any 
implementation on the real robot.**

Connection with the robot
----------

As we saw in Lab 1, one of the ways to connect to the robot is via Secure SHell (SSH). Thus, in this case, you need first to connect to the local Wi-Fi router (for further information please ask the Teaching Assistant) and then to the robot. To connect to the robot, you can open an SSH connection terminal, 

  .. code-block:: bash

    ssh X@10.40.2.21

and provide the password `X`. Since you are connected to the remote shell of the robot, you can navigate in the `~/catkin_ws` folder of its ROS workspace. To execute your locally saved ROS node, you need first to secure copy (`scp`) it in the robot's directory and then execute it. Thus, open a terminal in your VMware and execute,

  .. code-block:: bash

    scp `path_to_your_script` burger@10.40.2.21:/home/burger/catkin_ws/src/ee106s23/scripts/left_wall_following.py

Then, on the terminal that you have access on the burger (SSH), navigate to the `ee106s23` ROS package, and provide permission on the copied ROS node with the command `chmod +x left_wall_following.py`. To execute your ROS node on the Turtlebot3, perform the following command on the SSH terminal,

  .. code-block:: bash

    rosrun ee106s23 left_wall_following.py

To interrupt the behavior, you can cancel the execution of the ROS node in the same way as the Gazebo. In case you want to perform changes on your code, you can do this locally, and then copy back the new updated code on the robot. 

Communication with TurtleBot
----------------------------

- Once you have successfully login to the actual robot, 
  the following command can bring up the Kobuki mobile base. 

  .. code-block:: bash
    
    roslaunch turtlebot_bringup minimal.launch --screen

- Then you can open another terminal and remote login (again, twice) to the robot to run the script.

- Alternatively, you can use another terminal to run the teleop command for testing **using the default linear and angular velocity**. 

  .. code-block:: bash
    
    roslaunch turtlebot_teleop keyboard_teleop.launch

- To edit the script already copied to the robot, use the following command. 
  (This is where you may fail if ``-X`` option was not specified when using ssh.)

  .. code-block:: bash
    
    gedit ~/team01/turtlebot.py

- Then demo to TAs.

.. note::

  When you bring up the robot, the odometry will be reset (initialized to origin).

ROS Node template for the Left Wall-Following
----------

.. literalinclude:: ../scripts/left_wall_following.py
   :language: python

The above code can be used as a template for the Lab 7 and the final Lab. 

ROS Bag Recording and Data Logging
----------

One of the ways to record the data being produced or computed during a ROS scenario you can use the ROS bag command. Specifically, this command enables the ROS data logging feature, to capture information that is being published via the ROS Topics. This information is saved in ROS Bag files, which can be accessed at a later time and be replayed back to repeat the scenario. 

The ROS Bag recording can be used in our experiment to save the implemented scenario of the TurtleBot3 and then can be replayed to visualize the scenario. To save the ROS Bag you can execute while the robot is running,

  .. code-block:: bash

    rosbag record -a

The locally saved ROS Bag can be replayed back, by doing,

  .. code-block:: bash

    rosbag play name_of_the_rosbag.bag --clock -l
  
By using the `Space` button you can pause the replay. Additionally, by using the `rostopic list` command you can see that the captured ROS Topics are being replayed back. In our scenario, you will be asked to record a ROS Bag, to use after the lab to access and visualize the captured data from the real scenario in the Lab.



Submission
----------

The image below showcases the requirements of this lab, 

.. image:: ./pics/capstone_preview.png
    :align: center

#. Lab report with explanation and screenshots from the robot's navigation scenario.

#. Due time: 06/11/2023

#. Grading rubric:
      -  \+10% Communicate successfully with the real robot
      -  \+ 40%  Demo the task on the real robot
      -  \+10% Avoid collision with obstacles.
      -  \+10% Reach the goal area and kick the ball.
      -  \+30% Lab report with included ROS node code, screenshots of RViz by using the captured ROS Bag, showing the frames and the LiDAR visualization during the robot's navigation.

Lab Rules
---------

#. Safety is always the top priority.

   - No food or beverage allowed in the lab.
   - Report any suspicious cables, wires, etc.

#. Organize your station before you leave.

   - Organize wires, cables, etc.

#. Do not leave your personal information on the robot.

   - Create your own folder when you work, and delete code when you leave.
   - The robot is shared by two lab sections.

#. Do NOT make any changes to the wiring on the robot.

#. Please save the battery (recharging takes time), 
   and charge the robot if you do not have it running.
  
