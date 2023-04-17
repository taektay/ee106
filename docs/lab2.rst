Lab 2: ROS Tools and Testing on Real Robot
========================

Overview
--------

In this lab, we are going to test various ROS tools developed for illustrating the ROS Nodes and Topic graph relations, visualizing the ROS Topics in real-time, and saving-replaying ROS message data. ROS, as a tool-based framework, features a plethora of applications, which can also be seen `here <http://wiki.ros.org/Tools>`_.

Additionally, the Clearpath Jackal robot will be demonstrated in the class, along with the use of some of the ROS tools. Afterward, the simulated version of the Jackal robot will be deployed in the Gazebo simulator and it will be compared with the real robot. A 2D LiDAR ranging scenario will be demonstrated and will be the base of this lab's assignment.

Install Gazebo
-----------

The Gazebo simulator is a broadly used open-source robotic simulator, that has been used for ROS application development before testing on robots in the real world. To install the Gazebo simulator we need to perform,

.. code-block:: bash

  sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

To install the official ROS package of the Clearpath Jackal robot for its Gazebo simulation

.. code-block:: bash

  sudo apt-get install ros-noetic-jackal-gazebo

to install the official Gazebo package of the simulated Clearpath Jackal robot


ROS Launch
----------

Before we proceed with the experiment on the Jackal robot and the ROS tools, we need to describe the ROS launch file functionality. Particularly, a ROS launch file is a ``XML``  type file with a ``.launch`` file extension, which can be used to launch multiple ROS nodes at the same time, while setting values on parameters in the Parameters Server before ``rosrun``. The ROS launch files can are kept in the ``launch`` folder of each ROS package.

A normal content of a launch file may have,

.. code-block:: html

  <launch>
          <arg name="x_var" default="0" />
          <arg name="enable_var" default="true" />

          <!-- This is a comment -->
          <node pkg="package_name" name="node_name" type="node_filename_name" />

  </launch>

To execute the newly create ROS launch file, you can do,

.. code-block:: bash
  roslaunch package_name file.launch

Now, lets try to create a launch file for our created ROS package, namely the ``ee106s23``. Specifically, create a ROS launch that you can execute at the same time both the `publisher` and `subscriber` node of the Lab 1.

rqt and rqt_graph
----------

The rqt tool as a QT-based framework developed for ROS to enable the creation of user interface-enabled applications. The ``rqt_graph`` is visualizing tool that can illustrate the relations of the running ROS nodes and topics in a graph illustration.
To test the result of the rqt-graph, firstly enable the ROS nodes of your application and then execute the bellow command in a new terminal.

.. code-block:: bash

  rqt_graph

RViz : ROS Visualisation Tool
--------------

In addition, the most useful visualization tool that is integrated in ROS, is the RViz. This tool is used to illustrate in real-time the published raw information that is published via the ROS topics under a predefined coordinate system. To enable RViz you can perform in a separate terminal,

.. code-block:: bash

  rviz

Clearpath Jackal Robot and Gazebo Simulation
--------------

In order to start the Gazebo simulator with an empty world, you can execute,

.. code-block:: bash

  roslaunch gazero_ros empty_world.launch

To ideally exit or terminate Gazebo you should use the window terminating button. In many cases such as you close the terminal or Gazebo is not responding, you can execute in a separate terminal, 

.. code-block:: bash

  sudo killall gzserver
  sudo killall gzclient


then create a ros subscriber and try to collect the data from the raw pointcloud and check distances


and a new one, 

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

