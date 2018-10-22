/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node is used to open/close the box on buttonActivation
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>

#include <lib_modules/StampedBool.h>

bool buttonOn = false;
bool previousButtonState = false;
ros::Publisher box_pub ;

void buttonActivationCallback(const lib_modules::StampedBool::ConstPtr& msg ){
  std_msgs::Bool msgToBox;
  bool currentButtonState = msg->value;
  if (previousButtonState != currentButtonState){
      previousButtonState = currentButtonState;
      if (currentButtonState && !buttonOn){
	ROS_INFO("Button Ativated: Opening the Box");
	buttonOn = true;
        msgToBox.data = true;
	box_pub.publish(msgToBox);  
      } else if (currentButtonState && buttonOn){
	ROS_INFO("Button Desativated: Closing the Box");
        buttonOn = false;
        msgToBox.data = false;
	box_pub.publish(msgToBox);
      }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  box_pub = nh.advertise<std_msgs::Bool>("/BoxModule", 1000);

  ros::Subscriber sub = nh.subscribe("/ButtonModule_Blue", 100, buttonActivationCallback);
  ros::spin();

  return 0;
}
