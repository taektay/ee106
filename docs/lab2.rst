Lab 2: ROS Tools and Testing on a Real Robot
========================

Overview
--------

In this lab, we are going to test ROS tools that are developed for illustrating the ROS Nodes and Topic graph relations, visualizing the ROS Topics in real-time, and saving-replaying ROS message data. ROS, as a tool-based framework, features a plethora of applications, which can also be found `here <http://wiki.ros.org/Tools>`_.

Additionally, the `Clearpath Jackal <https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/>`_ mobile robot will be demonstrated in the class, along with the use of some of the ROS tools. Furthermore, the simulated version of the Jackal robot will be deployed in the Gazebo simulator and it will be compared with the real robot. A 2D LiDAR ranging scenario will be demonstrated and will be the base of this lab's assignment.

Installing Gazebo Simulator
-----------

The Gazebo simulator is a broadly used open-source robotic simulator, that has been used for ROS application development before testing on robots in the real world. To install the Gazebo simulator we need to perform,

.. code-block:: bash

  sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

To install the official ROS package of the Clearpath Jackal robot for its Gazebo simulation

.. code-block:: bash

  sudo apt-get install ros-noetic-jackal-gazebo

to install the official Gazebo package of the simulated Clearpath Jackal robot


ROS Launch files
----------

Before we proceed with the experiment on the Jackal robot and the ROS tools, we need to describe the ROS launch file functionality. Particularly, a ROS launch file is a ``XML``  type file with a ``.launch`` file extension, which can be used to launch multiple ROS nodes at the same time while it can set values on parameters in the Parameters Server before executing the ``rosrun``. The ROS launch files can are kept in the ``launch`` folder of each ROS package.

An example of a launch file could be,

.. code-block:: html

  <launch>
          <arg name="x_var" default="0" />
          <arg name="enable_var" default="true" />

          <!-- This is a comment -->
          <node pkg="package_name" name="node_name" type="node_filename" />

          <node pkg="package_name" name="node_name_2" type="node_filename_2 args="-x $(arg x_var) -enable_var $(arg enable_var)" />

  </launch>

where,

.. code-block:: html

  <arg name="x_var" default="0" />
  <arg name="enable_var" default="true" />

is the initialization of the variable ``x_var`` with the value 0 and the variable ``enable_var`` with True. The code part of,

.. code-block:: html

  <node pkg="package_name" name="node_name" type="node_filename" />

executes the node ``node_filename`` from the package ``package_name`` and names it ``node_name``. The second node call part, namely,

.. code-block:: html

  <node pkg="package_name" name="node_name_2" type="node_filename_2 args="-x $(arg x_var) -enable_var $(arg enable_var)" 

executes the ``node_filename_2`` node, but also provides argument information through the Parameter Server.

To execute the ROS launch file you can perform in a new terminal,

.. code-block:: bash

  roslaunch package_name file.launch

Now, lets try to create a launch file for our created ROS package, namely the ``ee106s23``. Specifically, create a ROS launch that you can execute at the same time both the `publisher` and `subscriber` node of the Lab 1. Show the results to the Teaching Assistant.

rqt and rqt_graph Tools
----------

The rqt tool as a QT-based framework developed for ROS to enable the creation of user interface-enabled applications. The ``rqt_graph`` is visualizing tool that can illustrate the relations of the running ROS nodes and topics in a graph illustration.
To test the result of the rqt-graph, firstly enable the ROS nodes of your application and then execute the bellow command in a new terminal.

.. code-block:: bash

  rqt_graph

RViz : ROS Visualisation Tool
--------------

In addition, the main visualization tool that is used in ROS software development, is the RViz. This tool is used to illustrate the raw information that is published by the ROS topics, in real-time, with respect to a predefined coordinate system. To enable RViz you can perform in a separate terminal,

.. code-block:: bash

  rviz

Gazebo Simulation and the Clearpath Jackal Robot
--------------

In order to start the Gazebo simulator with an empty world, you can execute, 

.. code-block:: bash

  roslaunch gazero_ros empty_world.launch

To properly exit or terminate Gazebo you should use the window terminating button. In many cases, such as closing abruptly the terminal or if the Gazebo is not responding, you can terminate it by executing in a new terminal, 

.. code-block:: bash

  sudo killall gzserver
  sudo killall gzclient

As the Gazebo is up and running, we can spawn a Jackal robot inside the simulated environment. To achieve that, we will create a dedicated ROS launch file in ``ee104s23/launch`` folder and attach the following,   

.. code-block:: html
  
  <launch>
    <arg name="x" default="0" />
    <arg name="y" default="0" />
    <arg name="z" default="1" />
    <arg name="yaw" default="0" />
    <arg name="joystick" default="true" />

    <!-- Configuration of Jackal which you would like to simulate.
        See jackal_description for details. -->
    <arg name="config" default="front_laser" />

    <!-- Load Jackal's description, controllers, and teleop nodes. -->
    <include file="$(find jackal_description)/launch/description.launch">
      <arg name="config" value="$(arg config)" />
    </include>
    <include file="$(find jackal_control)/launch/control.launch" />
    <include file="$(find jackal_control)/launch/teleop.launch">
      <arg name="joystick" value="$(arg joystick)" />
    </include>

    <!-- Spawn Jackal -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -model jackal -param robot_description -x $(arg x) -y $(arg y) -z $(arg z) -R 0 -P 0 -Y $(arg yaw)" />

  </launch>

.. then create a ROS subscriber and try to collect the data from the raw pointcloud and check distances

This file will be the ROS launch file that we will use to start Gazebo and spawn a simulated Jackal robot inside the world.  Specifically, the variables `x,y,z` and `yaw` define the initial position and orientation values of the robot in the world. The 'joystick' variable enables the teleoperation of the Jackal robot through a joystick controller. This file can be saved in a new ``.launch`` file inside the ``ee104s23`` ROS package. After the execution of the above ROS launch file, you can use ``rviz`` to visualize the capture sensory information from the simulated Jackal robot.

Robot Teleoperation
-----------------

.. rosrun, rostopic, rosmsg, rosnode, rosbag

Nowadays, most of the robots support teleoperation through a connected keyboard or a gamepad/joystick. In ROS we can use the ``teleop_twist_keyboard <https://github.com/ros-teleop/teleop_twist_keyboard>``_ to use our keyboard for robot teleoperation, which publishes `geometry_twist/Twist` message on the ``cmd_vel`` ROS topic. 

To install this package, 

.. code-block:: bash

  cd ~/catkin_ws/src/
  git clone https://github.com/ros-teleop/teleop_twist_keyboard.git

and build the catkin workspace.

.. Submission
.. -----------------

.. In this lab's submission, you need to develop a ROS node that can 

Reading Materials
-----------------

ROS Nodes
~~~~~~~~~

- `Understanding ROS Nodes <http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_

- `Initialization and Shutdown <http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown>`_

ROS Topics and Messages
~~~~~~~~~~~~~~~~~~~~~~~

- `Messages <http://wiki.ros.org/Messages>`_

- `Understanding ROS Topics <http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_

- `Publishers and Subscribers <http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers>`_

ROS Conventions
~~~~~~~~~~~~~~~

- `REP 103 Standard Units of Measure and Coordinate Conventions 
  <https://www.ros.org/reps/rep-0103.html>`_

- `REP 105 Coordinate Frames for Mobile Platforms <https://www.ros.org/reps/rep-0105.html>`_

