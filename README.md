# PFF_Submission
* NOTE:  Work completed upto question 6 Teleop Part
* Get user input from cin ( keypress similar to gaming can be done in a longer code)
* Pubs /cmd_vel topic to move
* 
*
* Drawing a square is executed as starting from a vertex, following a side till a dist of
* side-length is reached from initial vertex, turning 90 degrees, making current vertex as 
* base vertex and repeating.
* Subs /odom topic to get current pose data of the robot.
* Pubs /cmd_vel topic to move

Inside package pff_rohit

	urdf/diff_drive.urdf contains the urdf file for the robot.
	urdf/diff_drive_robot.pdf contains the link diagram for the robot.

	launch/diff_drive.launch conatins launch configuration file to open diff_drive.urdf using rviz

To view the robot in rviz,
	download the package

	add the following to ~/.bashrc

	source /opt/ros/kinetic/setup.bash
	export GAZEBO_MODEL_PATH=/home/user/PFF_Submission/src/
	(GAZEBO_MODEL_PATH refers to the location of workspace where you've kept the package)
	In terminal run the following:

	catkin_make
	roslaunch pff_rohit diff_drive.launch

	Once rviz is launched, set Fixed frame to body
	Go to 'Add' option and click on 'Robot Model'
	Zoom in to view the robot (size kept small so as to work on question 6)


To view the robot in gazebo,

	ros_launch pff_rohit diff_drive_gazebo.launch

To drive the robot in gazebo, after launching the robot using the above command, in a new terminal
	
	rosrun pff_rohit pff_rohit
	```
	'w' to drive straight
	'a' to turn left
	'd' to turn right
	'q' to quit
	```
robot_in_rviz.png shows how the robot will look inside rviz

![alt tag] (https://github.com/rohitsheth/PFF_Submission/blob/master/robot_in_rviz.png)


robot_in_gazebo.png shows how the robot in gazebo with empty world and a sun

![alt tag] (https://github.com/rohitsheth/PFF_Submission/blob/master/robot_in_gazebo.png)


Keypad_to_teleop.png shows update in cmd_vel topic when pressed different keys to drive the robot
![alt tag] (https://github.com/rohitsheth/PFF_Submission/blob/master/Keypad_to_teleop.png)
