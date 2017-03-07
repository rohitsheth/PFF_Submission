#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

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

  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w' to move forward, 'a' to turn left, 's' to stop	"
      "'d' to turn right, 'q' to exit.\n";

    //sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    ros::Rate loop_rate(10);
    char cmd;
    while(ros::ok()){

      std::cin >> cmd ;
      if(cmd!='w' && cmd!='a' && cmd!='d' && cmd!='q' && cmd!='s')
      {
        ROS_INFO("wrong Key! Press q to quit. Press s to stop \n");
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      //move forward
      if(cmd=='w'){
        base_cmd.linear.x = 0.25;
      }
      //turn left 
      else if(cmd=='d'){
        base_cmd.angular.z = 0.75;
      }
      //turn right
      else if(cmd=='a'){
        base_cmd.angular.z = -0.75;
      }
      //stop
      else if(cmd=='s'){
        base_cmd.linear.x = 0.0;
        base_cmd.linear.y = 0.0;
        base_cmd.linear.z = 0.0;
        base_cmd.angular.x = 0;
        base_cmd.angular.y = 0;
        base_cmd.angular.z = 0;
      }
      //quit
      else if(cmd=='q'){
        break;
      }

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
