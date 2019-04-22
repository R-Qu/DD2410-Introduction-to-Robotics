#### Assignment2

Download the file [Assignment_2_kinematics.zip](https://drive.google.com/file/d/1rWtw5aCTehons2TRChDsdTaCBcaOQ-0q/view?usp=sharing) and decompress it. Put the ROS package in your ROS workspace and run catkin_make. Remember to source the devel/setup.bash file if needed. The python file kinematics_assignment_metapackage/kinematics_assignment/scripts/IK_functions.py is the file you have to modify.

* Scalar Robotic:
A 3 DOF scara robot, with an inverse kinematic solution that is possible in analytic form.
![RL](https://github.com/R-Qu/DD2410-Robotics/blob/master/Assignment2%20Kinematics/kuka_info_large.png)

To visualize Scara, run: 

        roslaunch kinematics_assignment scara_launch.launch

* Kuka Robotic:
Solving this part will guarantee the best grade 
![RL](https://github.com/R-Qu/DD2410-Robotics/blob/master/Assignment2%20Kinematics/scara_explanation_large.png)

To visualize Kuka, run: 

        roslaunch kinematics_assignment kuka_launch.launch
