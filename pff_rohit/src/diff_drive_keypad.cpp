#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

/* This is the Diff_Drive Keypad Controller

		The controller takes input from the following keys to drive the robot
					w
				a s d

				w - forward
				a - left turn
				d - right turn
				s - stop

				Publishes message of the type: geometry_msgs::Twist 
				this topic is subscribed by the Gazebo Differential Drive plugin which changes
				joint states to drive the robot.

				Pressing q exits the keypad drive function and stops the node.

*/

/* Drive Class */
class Diff_Drive
{
private:
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_;

public:
	Diff_Drive(ros::NodeHandle &nh)
	{
		nh_ = nh;
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	/* Functionto drive robot from Keyboard */
	bool driveKeyboard()
	{
		std::cout << "Type a command and then press enter.  "
			"Use 'w' to move forward, 'a' to turn left, 's' to stop	"
			"'d' to turn right, 'q' to exit.\n";

		// sending commands of type "twist"
		geometry_msgs::Twist base_cmd;
		ros::Rate loop_rate(10);
		char user_key;
		while(ros::ok()){

			std::cin >> user_key ;
			if(user_key!='w' && user_key!='a' && user_key!='d' && user_key!='q' && user_key!='s')
			{
				ROS_INFO("Wrong Key! Press q to quit. Press s to stop \n");
				continue;
			}

			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			// move forward
			if(user_key=='w'){
				base_cmd.linear.x = 1.0;
			}
			// turn left 
			else if(user_key=='d'){
				base_cmd.angular.z = 1.0;
			}
			// turn right
			else if(user_key=='a'){
				base_cmd.angular.z = -1.0;
			}
			// stop
			else if(user_key=='s'){
				base_cmd.linear.x = 0.0;
				base_cmd.linear.y = 0.0;
				base_cmd.linear.z = 0.0;
				base_cmd.angular.x = 0;
				base_cmd.angular.y = 0;
				base_cmd.angular.z = 0;
			}
			// quit
			else if(user_key=='q'){
				break;
			}

			// published message on cmd_vel topic
			cmd_vel_pub_.publish(base_cmd);
			ros::spinOnce();
			loop_rate.sleep();
		}
		return true;
	}

};

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "diff_drive");
	ros::NodeHandle nh;

	Diff_Drive driver(nh);
	driver.driveKeyboard();
}
