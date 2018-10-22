/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node is used to retrive data from the joystick module 
               and publish them to the interface  
*/


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>
#include <string>

#include <lib_modules/Joystick.h>

ros::Publisher interface_pub;

void interfaceJoystickActivationCallback(const geometry_msgs::Vector3::ConstPtr& msg ){
  std_msgs::String msgToInterface;
  std::ostringstream ssmsg;
  ssmsg << "joystick"<<":"<<msg->x*0.002343-1.2<<":"<<1.2-msg->y*0.002343;
  std::string data(ssmsg.str());
  msgToInterface.data= data;
  interface_pub.publish(msgToInterface);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  interface_pub = nh.advertise<std_msgs::String>("/interface", 1000);
  ros::Subscriber sub = nh.subscribe("/JoystickModule", 100, interfaceJoystickActivationCallback);
  ros::spin();

  return 0;
}
