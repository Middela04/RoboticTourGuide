FRI Person Following Project
=============================

**Descripton**

This project enables the Building Wide Intelligence (BWI) robots at the University of Texas at Austin to follow people.
The Kinect_Code folder contains code which processes frames from the Azure Kinect camera to determine navigation requests that must be sent to the ROS side.
The fri_person_following folder is the ROS catkin project that services navigation requests from the Kinect code.

**Compilation**

For Kinect part, cd into build/src/ and run `make` to compile the program. If directory errors are prompted, delete the cache files in all folders and run `cmake ..`
The fri_person_following folder must be copied into your catkin_ws/src/ folder and built with `catkin build fri_person_following`

**Running**

The Kinect part and the ROS part contain their own main files. Because of versioning issues with ROS and Azure Kinect, both sides are connected using TCP.
The fri_person_following ROS project can be run with `rosrun fri_person_following fri_person_following_node` inside your workspace. The Kinect code can be run using `./KinectCode` inside build/src/. Note that the ROS part must run first followed by the Kinect part so the TCP connection is established properly. Conversely, the Kinect part should be terminated first for the ROS part of stop correctly.


**Interacting**

To be followed, the user should be in frame and raise their hand for 4 seconds or more for the robot to lock-on and follow you. Although localizing in a mapped enivronment is not required, for safer operation, it is recommended.
