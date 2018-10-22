/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node synchonises data retrived from multiple buttons
               and then updates the interface and controls the box Module  
*/


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sstream>
#include <string>

#include <lib_modules/StampedBool.h>

using namespace message_filters;

ros::Publisher interface_pub;
ros::Publisher box_pub ;


void interfaceButtonActivationCallback(const lib_modules::StampedBool::ConstPtr& redButton, 
  const lib_modules::StampedBool::ConstPtr& greenButton, const lib_modules::StampedBool::ConstPtr& yellowButton, 
  const lib_modules::StampedBool::ConstPtr& squaredGreenButton){
  std_msgs::String msgToInterface;
  std_msgs::Bool msgToBox;
  if ((redButton->value) || (greenButton->value) || (yellowButton->value) || (squaredGreenButton->value)){
    msgToInterface.data = "button:ON";
    msgToBox.data = true;
  }else{
    msgToInterface.data = "button:OFF";
    msgToBox.data = false;
  }
  box_pub.publish(msgToBox); 
  interface_pub.publish(msgToInterface);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  interface_pub = nh.advertise<std_msgs::String>("/interface", 1000);
  box_pub = nh.advertise<std_msgs::Bool>("/BoxModule", 1000);

  std::cout<<"m init"<<std::endl;

  message_filters::Subscriber<lib_modules::StampedBool> redButton_sub(nh, "/ButtonModule_Blue", 1);
  message_filters::Subscriber<lib_modules::StampedBool> greenButton_sub(nh, "/ButtonModule_Green", 1);
  message_filters::Subscriber<lib_modules::StampedBool> yellowButton_sub(nh, "/ButtonModule_Yellow", 1);
  message_filters::Subscriber<lib_modules::StampedBool> squaredGreenButton_sub(nh, "/ButtonModule_SquaredGreen", 1);

  typedef sync_policies::ApproximateTime<lib_modules::StampedBool, lib_modules::StampedBool, 
                                          lib_modules::StampedBool, lib_modules::StampedBool> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), redButton_sub, yellowButton_sub, squaredGreenButton_sub,greenButton_sub);
  sync.registerCallback(boost::bind(&interfaceButtonActivationCallback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
