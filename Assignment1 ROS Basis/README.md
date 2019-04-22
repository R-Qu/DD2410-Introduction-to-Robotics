#### Assignment 1
This programming exercise will help you to get acquainted with ROS and several development tools that you will use throughout the course.The assignment is based on a simulated Kobuki robot from the Yujin robotics company. The final goal of the programming assignment consists in controlling a simulated two-wheeled differential drive robot Kobuki and make it follow a wall by using readings from distance sensors mounted on the robot.

This assignment is divided in several sections to help guide you and so that you can get familiarized with the ROS and Linux environment that you will use throughout the course.
- Installing Ubuntu 16.04 (64 bit) and ROS Kinetic on your computer (either via Virtualbox or via a dual-partition) or use the computers in the computer rooms
- Ubuntu/Linux tutorials for those of you who are not very experienced using Ubuntu/Linux
- ROS (Kinetic) tutorials
- Description of the robot setup -- the Kobuki robot + distance sensors + DC motors & encoders

#### ROS website tutorials
Please go through [the beginner level ROS tutorials](http://wiki.ros.org/ROS/Tutorials)

Skip anything that has to do with the rosbuild system for ROS versions Groovy and earlier. In the course we will use the ROS Kinetic distribution and we will use the catkin build system.The beginner tutorials which are crucial for the exercise are numbers: 2-10, 12-13. Make sure to select catkin in each of the tutorials.

Extra: it is also good to look at tutorial 1 of [the intermediate level ROS tutorial](http://wiki.ros.org/ROS/Tutorials#Intermediate_Level)

#### The Kobuki robot
The Kobuki robot is a 2 wheeled mobile robot. You can find information about the robot's packages that are available in ROS at [the Kobuki ROS wiki page](http://wiki.ros.org/kobuki).

Launching Kobuki in ROS
Follow [tutorial](http://wiki.ros.org/kobuki/Tutorials/Simulated%20Kobuki%20navigation%20in%20perfect%20world) for instructions on how to launch a simulated Kobuki, visualize it using Rviz and controlling it using your keyboard.
