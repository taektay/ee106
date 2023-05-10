Lab 5: ROS Services, Custom Service Messages, and Overview
====================

Overview
--------

In this lab, we will expand our knowledge working with ROS by using the ROS Services, and we will compare it with the use ROS Topics of the previous labs. In addition, we will create two new ROS Nodes that will communicate with a ROS Service call and in the end we will create a new customized ROS Service type.

ROS Services: Request and Response
----------

As seen in the previous labs, ROS Topics are used to stream data between ROS Nodes, to help in their commucation and exhange of information to perform a more complex task. ROS Services represent also a way of communication within ROS Nodes, but they are mainly based on request and reply interactions. 

Specifically, a ROS Service is described by specific data fields, as a type of a ROS Topic, however, there is distinction between request and response fields that use different parameters. For example, the `rospy_tutorials/AddTwoInts` is a ROS Service that has been created by ROS Python online tutorials to demonstrate ROS service request of adding two different numbers. 

.. code-block:: bash

    int64 a
    int64 b
    ---
    int64 sum


Initially, the separation line `--` disticts the request and response parameters. In our example, the `int64` ``a`` and `int64` ``b`` are the parameters of the request of the ROS service, that need to be set before the service call. With the service call, the ROS node that will implement the ROS Service callback function, will be triggered and will fillup the response parameters, which will be 
As it is obvious, the request and response variable can be more than one, according to our request and response variable need.

Additionally, ``rosservice`` bash command can be used in the same way as the ``rostopic`` command, to preview further information about the available ROS Services. For example, argument ``list`` is used to list all the available ROS Services,

.. code-block:: bash

    rosservice list

The `show` argument is used to show 

.. code-block:: bash

    rosservice echo rospy_tutorials/AddTwoInts

To preview the full use of the ``rosservice`` command you can use the ``help`` argument, such as,

.. code-block:: bash

    rosservice help

Notably, the ``rossrv`` bash command is used to 

 the  available ROS Services can b

.. code-block:: bash

    rossrv show 
    

ROS Service Use Example
----------

Let's see an example by using the `rospy_tutorials/AddTwoInts` ROS Service. First, create a new ROS Publisher and Subcriber node, as followed on our `ROS tutorial`_, and name them `node_a.py` and `node_b.py` under the ``ee106s23`` ROS package. In addition, modify both the ``node_a`` and ``node_b`` to the following,

.. code-block:: python

    #!/usr/bin/env python3

    import rospy
    from std_msgs.msg import String, Header
    from std_srvs.srv import SetBool
    from rospy_tutorials.srv import  AddTwoInts

    def talker():

        pub = rospy.Publisher('chatter', String, queue_size = 10)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()

        content = "welcome to the Robotics Lab " + str(header.stamp)
        pub.publish(content)
        rate.sleep()

    def service_caller():
            
            rospy.wait_for_service('add_two_ints')
            
            add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            try:
                resp1 = add_two_ints(2, 1)
                print(resp1)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))


    if __name__ == '__main__':
        try:
            rospy.init_node('node_a')
            service_caller()
        except rospy.ROSInterruptException:
            pass

and 

.. code-block:: python

    #!/usr/bin/env python3

    import rospy
    from std_msgs.msg import String
    from rospy_tutorials.srv import  AddTwoInts

    def callback(data):
        rospy.loginfo(data.data)

    def add_two_ints(req):
        print(req)
        return (req.a + req.b)
        
    def listener():
        rospy.init_node('node_b')
        rospy.Subscriber('chatter', String, callback)
        rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
        rospy.spin()

    if __name__ == '__main__':
        listener()

Creation of new ROS Service Type
----------


