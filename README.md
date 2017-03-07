# PFF_Submission
* NOTE:  Currently working on Question 6. Below is the working till question 5. 
* NOTE: Robot drives using the keyoard, though it is osciallating as I forgot to add the caster wheel.


Inside package pff_rohit

	urdf/diff_drive.urdf contains the urdf file for the robot.
	urdf/diff_drive_robot.pdf contains the link diagram for the robot.

	launch/diff_drive.launch conatins launch configuration file to open diff_drive.urdf using rviz

To view the robot in rviz,
	download the package inside workspace folder
	run the following commands in terminal:

	source devel/setup.bash
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
Diff_drive_robot.png shows how the robot will look inside rviz

![alt tag] (https://github.com/rohitsheth/PFF_Submission/blob/master/Diff_drive_robot.png)
