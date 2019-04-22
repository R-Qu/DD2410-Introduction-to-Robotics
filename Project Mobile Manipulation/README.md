#### Project

For the project to be run in personal computers, your own missing system dependencies have to be met, which can be done through the instructions in the [README](https://github.com/kth-ros-pkg/Robotics_intro/blob/master/README.md) in the repository.

#### Instructions:

Download [the packages](https://github.com/kth-ros-pkg/Robotics_intro):

$ cd ~/catkin_ws/src/

$ git clone https://github.com/kth-ros-pkg/Robotics_intro.git

Build the project:

$ cd ~/catkin_ws

$ catkin_make -DCATKIN_ENABLE_TESTING=0

$ source devel/setup.bash

You can either clone the github repository or install the zip file. However, you are encouraged to get the repository since it will be updated if some bugs/missing components/other problems are encountered, while the zip will only receive major updates and you will have to merge things manually in your computer.

#### Launch the simulation
Run the command "roslaunch robotics_project launch_project.launch" from a terminal to launch the system in Gazebo and RVIZ. If you do not want to run the Gazebo graphical interface (client) in order to speed things up, you can set the "gzclient" variable in the launch file to false.

If everything works out, you should see the robot moving around the appartment, folding its arm, approaching a chair and lowering its head. This example shows how to call services and actions and interact with topics from your state machine. Remember though that you can do all things from the command line, which should help you design the right sequence of states faster.

#### Dealing with probabilities
As a some of the components in the system are not deterministic but work with probabilistic models to approximate the behaviour of the robot or the environment, there is a chance the simulation might fail despite the fact that all components are correctly implemented and executed. This is perfectly fine and will still account as a valid solution. Some examples of this could be the [AMCL](http://wiki.ros.org/amcl) localization failing to converge toward the actual robot pose, the robot dropping the cube while carrying it or [MoveIt!](https://moveit.ros.org/) computing a non-valid trajectory for the robot arm. You are welcome to find more sources of error and introduce them to us!

In order to deal with this, you are expected to implement feedback (and maybe some error handling?) in your state machine, so that when some of the errors mentioned above occur, your system can identify them and act consequently. A basic example of this could be to log or print in terminal the feeback from calling an action server or the result from a service call.

#### Launch the system: 

$ roslaunch robotics_project gazebo_project.launch

$ roslaunch robotics_project launch_project.launch 

Now Gazebo and the rest of the system will be running in two different terminals.

#### Relaunch the system: 

Kill (Ctrl-C) only the terminal with the launch_project.launch 
Delete manually the "tiago_steel" and "aruco_cube" models from Gazebo, as shown here (Links to an external site.)Links to an external site.. (Unless they've been deleted automatically).
Relaunch the "robotics_project launch_project.launch" as before.
The bug!

If you encounter this warning after killing the system 

"[WARN] Controller Spawner couldn't find the expected controller_manager ROS interface."

It means things have gone wrong and then you need to kill both terminals and relaunch them as explained above.



 

 

 

 
