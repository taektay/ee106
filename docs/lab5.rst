Lab 5: ROS Services, Custom Service Messages, and Overview
====================

Overview
--------

In this lab, we will expand our knowledge of working with ROS by using ROS Services, and we will compare it with the use of ROS Topics in the previous labs. In addition, we will create two new ROS Nodes that will communicate with a ROS Service call, and in the end, we will create a new customized ROS Service type.

ROS Services: Request and Response
----------

As seen in the previous labs, ROS Topics are used to stream data between ROS Nodes to help in their communication and exchange of information to perform a more complex task. ROS Services represent also a way of communication within ROS Nodes, but they are mainly based on request and response interactions. 

Specifically, a ROS Service is described by specific data fields, as a type of a ROS Topic, however, there is a distinction between request and response fields that use different parameters. For example, the `rospy_tutorials/AddTwoInts` is a ROS Service that has been created by ROS Python online tutorials to demonstrate the ROS service request of adding two different numbers. 

.. code-block:: bash

 int64 a
 int64 b
 ---
 int64 sum


Initially, the separation line `--` separates the request and response parameters. In our example, the `int64` ``a`` and `int64` ``b`` are the parameters of the request of the ROS service, that need to be set before the service call. Thus, with the service call, the ROS node that has initialized the ROS Service triggers its function and forms the response variables, which will be returned. On the service caller side, the caller node will get back the output (response) of the ROS Service call and will store it locally to be accessed.
As it is obvious, the request and response variables can be more than one, depending on the used ROS Service.

The ``rosservice`` bash command can be used in the same way as the ``rostopic`` command, to preview further information about the available ROS Services. For example, the argument ``list`` is used to list all the available ROS Services,

.. code-block:: bash

 rosservice list

The `info` argument is used to show information about the selected ROS Service, i.e. `add_two_ints`.

.. code-block:: bash

 rosservice info add_two_ints

To preview the full use of the ``rosservice`` command you can use the ``help`` argument, such as,

.. code-block:: bash

 rosservice help

Notably, the ``rossrv`` bash command is used to display information about ROS Service types, such as,

.. code-block:: bash

 rossrv show rospy_tutorials/AddTwoInts
 

ROS Service Use Example
----------

Let's see an example by using the `rospy_tutorials/AddTwoInts` ROS Service. First, create a new ROS Publisher and Subscriber node, as followed on our `ROS tutorial <https://ucr-robotics.readthedocs.io/en/latest/intro_ros.html>`_ and name them `node_a.py` and `node_b.py` under the ``ee106s23`` ROS package. In addition, modify both the ``node_a`` and ``node_b`` to the following,


.. code-block:: python  
 :emphasize-lines: 5,20,21,22,23,24,25,26,27
 
 #!/usr/bin/env python3

 import rospy
 from std_msgs.msg import String, Header
 from rospy_tutorials.srv import AddTwoInts

 def talker():

    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rate = rospy.Rate(10) # 10hz

 while not rospy.is_shutdown():
    header = Header()
    header.stamp = rospy.Time.now()

    content = "welcome to the Robotics Lab " + str(header.stamp)
    pub.publish(content)

    # Call of the ROS Service 'add_two_ints'
    rospy.wait_for_service('add_two_ints')
    
    add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    try:
        response_msg = add_two_ints(2, 1)
        print(response_msg)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    rate.sleep()


 if __name__ == '__main__':
    try:
        rospy.init_node('node_a')
        talker()
    except rospy.ROSInterruptException:
        pass

and the ``node_b``,

.. code-block:: python
 :emphasize-lines: 5,11,12,13,19

 #!/usr/bin/env python3

 import rospy
 from std_msgs.msg import String
 from rospy_tutorials.srv import AddTwoInts

 def callback(data):
    rospy.loginfo(data.data)

 # ROS Service function to be executed when the service is called. The return will provide the response of the service to the caller.
 def add_two_ints(req):
    print(req)
    return (req.a + req.b)
 
 def listener():
    rospy.init_node('node_b')
    rospy.Subscriber('chatter', String, callback)
    # Initialization of the ROS Service
    rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
    rospy.spin()

 if __name__ == '__main__':
    listener()

As it is evident, the highlighted code are new additions in our ROS publisher-subscriber nodes, to integrate the creation (`node_b`) and the call (`node_a`) of the ROS Service.

Creation of new ROS Service Type
----------

For the generation of the new ROS Service type, initially, we create a new folder ``srv`` inside the `ee106s23` ROS package. Then, we create a ``ee106s23_service.srv`` file, which will contain the main request/response data structure,

.. code-block:: python

 string request_msg
 ---
 string response_msg


.. Our main goal for this service is transit a message 

To use the new ROS Service we have to build the workspace, but first, we need to apply the modifications of the `CMakeLists.txt <https://ucr-ee106.readthedocs.io/en/latest/lab1.html#creation-of-custom-ros-message>`_. Also, we have to uncomment the following part in the `CMakeLists.txt` file to enable the build of our newly created ROS Service, 

.. code-block:: python

 # Declare the service files to be built
 add_service_files(FILES
    ee106s23_service.srv
 )

As we have applied all the changes, we build the workspace and we can use our new ROS Service in any ROS Node, as the package dependencies are satisfied.

Reading Materials
-----------------

ROS Services
~~~~~~~~~~~~~~~

- `ROS Services, Types, Tools (Python) <http://wiki.ros.org/rospy/Overview/Services>`_

- `Custom ROS Services and Service Description Specification <http://wiki.ros.org/srv>`_

.. Submission
.. ----------

.. #. Submission: individual submission via Gradescope

.. #. Due time: 11:59pm, May 23, Tuesday

.. #. Files to submit: 

..    - lab5_report.pdf including the developed ROS Node

.. #. Grading rubric:

..    + \+ 10% Create a new folder in ``ee106s23`` ROS package, by using the name ``worlds``. Download the `Gazebo world <>`_ and copy it inside the ``ee106s23/worlds/`` folder.
..    + \+ 10% Create the above ROS launch file and copy it inside the ``ee106s23/launch/``, under the name of ``lab5_submission.launch``. Execute the launch file in a separate terminal. As this is completed, launch the ``turtlebot3.launch`` file and showcase the spawn of the robot inside the created world. 
..    + \+ 10% Broadcast 3 new ROS frames (similar to Lab 3) that will be placed on the front, left, and right bumper. Please ensure that the frames are properly placed at the correct positions, by checking also through RViz.
..    + \+ 10% Create a ROS Node that will contain a ROS subscriber to the robot's onboard LiDAR module, a ROS publisher on the ``cmd_vel`` ROS Topic of type `Twist`.
..    + \+ 10% Include 3 ROS listeners of capturing the transformation of the LiDAR module and the three bumper frames of the robot.
..    + \+ Transform all `non-inf` ranging measurements to all the 3 bumper coordinate systems and form the criticality levels for each of them
..    + \+ 10% Develop the left wall-following technique. Hint: The robot goes fowrward as long as the distance of the left bumper is within a fixed distance (i.e. 0.3 < r < 0.5). In case the robot gets in a smaller distance than the minimum distance, it has to correct its orientation (clockwise rotation) to continue without colliding. In the same sense, if the robot gets further than the maximum distance, it has perform an anti-clockwise rotation to approach more the following left wall. All that time, you need to check that the robot is not approaching the right wall too, by applying the corresponding actions. In case the robot reaches a spot where front distance is closer than a specific range and the left wall 
..    + \- 15%  Penalty applies for each late day. 


