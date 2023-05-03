Lab 4: Open Loop Control
========================

Overview
--------

In this lab, we are going to learn how to write a Python script to control the Turtlebot3 robot. Specifically, the task is to make the robot move in a square shape using open-loop control 
(i.e. sending commands only; no feedback). 
The waypoints to visit are [4, 0], [4, 4], [0, 4] and [0, 0]. 
In other words, the robot should move forward 4 meters, turn left 90 degrees, 
move forward again 4 meters, and so on, until going back to the origin. 
Note that the robot is supposed to stop at the origin after completing this square movement,
and the Python script should exit gracefully. 




.. Submission
.. ----------

.. #. Submission: individual submission via Gradescope

.. #. Demo: required during the lab session (will use autograder; see below)

.. #. Due time: 5:00pm, Oct 14, Friday

.. #. Files to submit: (please use exactly the same filename; case sensitive)

..    - lab2_report.pdf
..    - open_loop.py

.. #. Grading rubric:

..    + \+ 50%  Clearly describe your approach and explain your code in the lab report.
..    + \+ 40%  The robot can visit all four vertices of the square trajectory (error < 1.0m). 
..      Partial credits will be given according to the number of vertices visited.
..    + \+ 10%  The script can complete the task on time and exit gracefully.
..    + \- 15%  Penalty applies for each late day. 

.. Autograder
.. ----------

.. All code submissions will be graded automatically by an autograder uploaded to Gradescope.
.. Your scripts will be tested on a Ubuntu cloud server using a similar ROS + Gazebo environment.
.. The grading results will be available in a couple of minutes after submission.

.. The autograder works in the following way (no action on your side needed; just to explain). 
.. (1) Under Gazebo simulation environment, the submitted Python script will be run for once 
.. and the robot trajectory will be saved into csv files. 
.. (2) The scores will be given by evaluating the saved trajectory and uploaded back to Gradescope.


.. Testing parameters are as follows. 

.. #. The tolerance for distance error is set to 1.0m (considering this is open-loop control).

..    - For example, passing point [3.6, 3.4] is approximately equivalent to passing point [4.0, 4.0].

.. #. The time limit for the submitted script is set to 5 mins.

..    - If running properly, the task in this lab can be done in about 1 min, based on our testing.
..    - If running timeout, the script will be terminated and a 10% penalty will apply.
..    - Therefore, it is important that your script can exit gracefully after task completion.
..      (Just avoid using infinite loops and/or remember to add a break condition.)

.. #. The global time limit on Gradescope server is set to 10 mins. 

..    - If running timeout, the entire grading process will be terminated and you will have no grading results. 
..    - This can happen if you have dead loop in the script (e.g., ``while True: xxx``)
..      and the autograder is not able to terminate the script. 
..      (Scripts like this cannot be terminated by ``Ctrl + C`` in terminals, if you test it yourself.)