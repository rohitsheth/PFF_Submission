#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"

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

static geometry_msgs::Pose robot_pose;

class Diff_Drive
{
private:
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber odom_subs_ ;

public:
	Diff_Drive(ros::NodeHandle &nh)
	{
		nh_ = nh;
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	static void callback(const nav_msgs::Odometry::ConstPtr& msg){
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  robot_pose = msg->pose.pose ;
	}
	/* Function to drive robot from Keyboard */
	bool driveKeyboard()
	{
		std::cout << "Type a command and then press enter.  "
			"Use 'w' to move forward, 'a' to turn left, 's' to stop	"
			"'d' to turn right, 'q' to exit.\n";

		// sending commands of type "twist"
		geometry_msgs::Twist base_cmd;
		ros::Rate loop_rate(1000);
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

	/* Functionto drive robot in circle */
	bool driveCircle()
	{
		std::cout << "The working of the function is trivially similar to that of square function\nI have an internship and research work and since it is an optional question, I've decided to skip it.\n";
		return true;
	}

		/* Function to drive robot in a square */
	bool driveSquare()
	{
	
		double side;
		double distsq = 0;
		geometry_msgs::Twist base_cmd;
		geometry_msgs::Pose init_pose ;
		ros::Rate loop_rate(1000);
		bool turn = false;

		std::cout << "Enter the side of square (0<side<5)\n";
		std::cin >> side;
		
		odom_subs_ = nh_.subscribe("odom", 1000, callback);
		init_pose = robot_pose;

		while(ros::ok()){
			
			if( (side < 0 ) || (side > 5) ){
				ROS_INFO("Wrong Value! Enter side again. \n");
				std::cin >> side;
			}
			// get distance
			distsq = 0;
			distsq += (init_pose.position.x - robot_pose.position.x)*(init_pose.position.x - robot_pose.position.x) ;
			distsq += (init_pose.position.y - robot_pose.position.y)*(init_pose.position.y - robot_pose.position.y) ;
			distsq += (init_pose.position.z - robot_pose.position.z)*(init_pose.position.z - robot_pose.position.z) ;
			
			// check if a corner point is reached, if yes turn
			if(distsq >= (side*side) ){
				turn = true;
				std::cout << "now turning\n" ;
				base_cmd.linear.x = 0;
			}

			if(!turn){				
				base_cmd.linear.x = 0.1;
			}
			else{
				// check if 90 degree turn is complete, if yes, turn complete and proceed straight
				if( ( (init_pose.orientation.w - robot_pose.orientation.w ) <= 0.7071) && (init_pose.orientation.z - robot_pose.orientation.z ) <= 0.707  ){
					base_cmd.angular.z = 0.1;
				}
				else{
					//  re-init ini_pose as current pose and repeat algorithm to continue drawing squares
					init_pose = robot_pose;
					turn = false;
				}
			} 
			// published message on cmd_vel topic
			odom_subs_ = nh_.subscribe("odom", 1000, callback);
			
			cmd_vel_pub_.publish(base_cmd);
			ros::spinOnce();
		}
			return true;
	}
};

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "diff_drive");
	ros::NodeHandle nh;
	char option;

	Diff_Drive driver(nh);
	std::cout << "\nEnter the one of the following options:\n1. Keypad\n2. Circle(I've skipped this due to other priorities)\n3. Square\n\n" ;
	std::cin >> option ;
	
	if(option=='1'){
		driver.driveKeyboard();
	}
	else if(option=='2'){
		driver.driveCircle();
	}
	else if(option=='3'){
		driver.driveSquare();
	}
	else{
		std::cout << "\nWrong option. Please restart node\n";
	}
}
