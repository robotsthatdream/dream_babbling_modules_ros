/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node is used to start/stop the thymio on buttonActivation
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>

#include <lib_modules/StampedBool.h>

bool buttonOn = false;
bool previousButtonState = false;

void buttonActivationCallback(const lib_modules::StampedBool::ConstPtr& msg ){
  bool currentButtonState = msg->value;
  if (previousButtonState != currentButtonState){
      previousButtonState = currentButtonState;
      if (currentButtonState && !buttonOn){
	ROS_INFO("Button Ativated: Sending the Run CMD To Thymio");
	buttonOn = true;
        system("./scpSendRun.sh");
        ROS_INFO("Run CMD Executed");
  	    //system(THYMIO_CONTROL_SCRIPT 'run');
      } else if (currentButtonState && buttonOn){
	ROS_INFO("Button Desativated: Sending the Stop CMD To Thymio");
        buttonOn = false;
        system("./scpSendStop.sh");
        ROS_INFO("Stop CMD Executed");
	      // system(THYMIO_CONTROL_SCRIPT 'stop');
      }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ButtonModule_Blue", 100, buttonActivationCallback);
  ros::spin();

  return 0;
}
