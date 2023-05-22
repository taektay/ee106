Lab 7: Wall Following and the Finite-State Machine
====================

Overview
--------

In this lab, we will continue the development of the Turtlebot3 left wall-following behavior, by fully describing the navigation and sensing problem and the way of approaching it. Additionally, we will introduce the Finite State Machine (FSM) which will be integrated for the development of our behavior. 

The developed behavior will be evaluated and tested on the real Turtlebot3 Burger on Week 10, in a real wall-following setting.

Left Wall-following Scenario
--------

To perform the left wall-following initially, we have to consider the type of sensor modules that the robot will need. In general, there are many types of sensors that can indicate the existence of a wall around the robot, either with physical touch or by distance. In our case, we place a LiDAR sensor on the top of the Turtlebot3 Burger, which provides `360°` range measurements around it. For the left wall-following scenario, we can reduce the captured ranging measurements down to the `front` and `left` side of the robot. In many robots, an ultrasonic ranging sensor is used to provide distances of the front and left side of the robot, but in our case, the LiDAR can provide a range of distance measurements on the front left side of the robot.

The cases that a robot might experience during its exploration in a maze, by following left wall-following, are depicted below,

.. image:: ./pics/wall_following.png
 :width: 400
 :align: center

Initially, as can be seen from the image, every case can be modeled by using two parameters, the `Left Side` and the `Right Side` occupancy. Thus, the robot depending on the values of these two parameters can decide on the next action-move, which can be moving `forward`, `left`, or `right`. These cases can be represented also as the following lookup matrix,

.. list-table:: 
  :align: center
  :widths: 50 50 50
  :header-rows: 1

  * - Left Side
  - Right Side
  - Action
  * - Free
  - Free
  - Left
  * - Free
  - Occupied
  - Right 
  * - Occupied
  - Free
  - Forward
  * - Occupied
  - Occupied
  - Right



Finite-State Machine (FSM)
--------

A Finite-State Machine is a state automata that can formulate an algorithmic problem, which can be described by distinct states and transitions between them. Specifically, an FSM contains a finite number of states with an initial starting state, and at each moment only one state can be active in the machine. To have a transition between states, specific actions needed to occur (triggered) that also are described by the transition actions.

.. image:: ./pics/fsm.png
 :width: 400
 :align: center

The left wall-following problem described above can be illustrated as a FSM, by using the robot actions as the states and the range measurements as the transition triggers. As the FSM is formed it can be integrated inside the motion planning ROS node, to perform left wall following. Notably, to achieve the robot's inclination towards the left wall while moving forward, an `extreme_close_to_wall` parameter is used, which can be enabled when the robot has the left wall less than `10cm` closer to its left side.

Submission
--------

#. Group Submission (2-people)

#. Due time: 11:59, June 4, Sunday

#. Files to submit:

 - lab7_report.pdf with link of the video/s included. Please provide a report describing all the following steps and results experienced in both experiments.

#. Grading rubric:

 + \+ 10% Download the two new Gazebo worlds, namely `complex.launch <>`_ and `more_complex.launch <>`_ and place them inside the `worlds` folder of ``ee106s23``. Update the `lab5_turtlebot_world.launch` file to load the new worlds, for each experiment. 
 + \+ 40% Fully integrate the FSM behavior in the Turtlebot3 Burger motion planning behavior, based on the work of the ROS node of `Lab 5 <>`_. 
 + \+ 25% Demonstrate the left wall-following behavior on the `complex_case.world`, provide comments about the robot behavior, and provide a panoramic video of the result (link).
 
 .. image:: ./pics/complex_case.png
 :align: center

 + \+ 25% Demonstrate the left wall-following behavior on the `more_complex_case.world`, provide comments about the robot behavior, and include a panoramic video of the result in the report (link).
 
 .. image:: ./pics/more_complex_case.png
 :align: center

 + \- 15% Penalty applies for each late day. 


Information
--------
`Finite-state Machines <>`_ https://en.wikipedia.org/wiki/Finite-state_machineLab 7: Wall Following and the Finite-State Machine
====================

Overview
--------

In this lab, we will continue the development of the Turtlebot3 left wall-following behavior, by fully describing the navigation and sensing problem and the way of approaching it. Additionally, we will introduce the Finite State Machine (FSM) which will be integrated for the development of our behavior. 

The developed behavior will be evaluated and tested on the real Turtlebot3 Burger on Week 10, in a real wall-following setting.

Left Wall-following Scenario
--------

To perform the left wall-following initially, we have to consider the type of sensor modules that the robot will need. In general, there are many types of sensors that can indicate the existence of a wall around the robot, either with physical touch or by distance. In our case, we place a LiDAR sensor on the top of the Turtlebot3 Burger, which provides `360°` range measurements around it. For the left wall-following scenario, we can reduce the captured ranging measurements down to the `front` and `left` side of the robot. In many robots, an ultrasonic ranging sensor is used to provide distances of the front and left side of the robot, but in our case, the LiDAR can provide a range of distance measurements on the front left side of the robot.

The cases that a robot might experience during its exploration in a maze, by following left wall-following, are depicted below,

.. image:: ./pics/wall_following.png
 :width: 400
 :align: center

Initially, as can be seen from the image, every case can be modeled by using two parameters, the `Left Side` and the `Right Side` occupancy. Thus, the robot depending on the values of these two parameters can decide on the next action-move, which can be moving `forward`, `left`, or `right`. These cases can be represented also as the following lookup matrix,

.. list-table:: 
 :align: center
 :widths: 50 50 50
 :header-rows: 1

 * - Left Side
 - Right Side
 - Action
 * - Free
 - Free
 - Left
 * - Free
 - Occupied
 - Right 
 * - Occupied
 - Free
 - Forward
 * - Occupied
 - Occupied
 - Right



Finite-State Machine (FSM)
--------

A Finite-State Machine is a state automata that can formulate an algorithmic problem, which can be described by distinct states and transitions between them. Specifically, an FSM contains a finite number of states with an initial starting state, and at each moment only one state can be active in the machine. To have a transition between states, specific actions needed to occur (triggered) that also are described by the transition actions.

.. image:: ./pics/fsm.png
 :width: 400
 :align: center

The left wall-following problem described above can be illustrated as a FSM, by using the robot actions as the states and the range measurements as the transition triggers. As the FSM is formed it can be integrated inside the motion planning ROS node, to perform left wall following. Notably, to achieve the robot's inclination towards the left wall while moving forward, an `extreme_close_to_wall` parameter is used, which can be enabled when the robot has the left wall less than `10cm` closer to its left side.

Submission
--------

#. Group Submission (2-people)

#. Due time: 11:59, June 4, Sunday

#. Files to submit:

 - lab7_report.pdf with link of the video/s included. Please provide a report describing all the following steps and results experienced in both experiments.

#. Grading rubric:

 + \+ 10% Download the two new Gazebo worlds, namely `complex.world <https://github.com/UCR-Robotics/ee106/blob/main/scripts/complex.world>`_ and `more_complex.world <https://github.com/UCR-Robotics/ee106/blob/main/scripts/more_complex.world>`_ and place them inside the `worlds` folder of ``ee106s23``. Update the `lab5_turtlebot_world.launch` file to load the new worlds, for each experiment. 
 + \+ 40% Fully integrate the FSM behavior in the Turtlebot3 Burger motion planning behavior, based on the work of the ROS node of `Lab 5 <https://ucr-ee106.readthedocs.io/en/latest/lab5.html#submission>`_. 
 + \+ 25% Demonstrate the left wall-following behavior on the `complex_case.world`, provide comments about the robot behavior, and provide a panoramic video of the result (link).
 
 .. image:: ./pics/complex_case.png
 :align: center

 + \+ 25% Demonstrate the left wall-following behavior on the `more_complex_case.world`, provide comments about the robot behavior, and include a panoramic video of the result in the report (link).
 
 .. image:: ./pics/more_complex_case.png
 :align: center

 + \- 15% Penalty applies for each late day. 


Reading Materials
--------

Wikipedia `Finite-state Machines <https://en.wikipedia.org/wiki/Finite-state_machine>`_ 