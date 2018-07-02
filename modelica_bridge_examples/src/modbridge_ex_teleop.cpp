// example node for running joystick values

#include <ros/ros.h>
#include "modelica_bridge/ModComm.h"
#include <sensor_msgs/Joy.h>

ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  modelica_bridge::ModComm springVal;
  for(int i = 0; i < 8; i++) {
    springVal.data.push_back((i+1)*joy->axes[i]);
  }
  springVal.size = springVal.data.size();
  pub.publish(springVal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modbridge_examples_teleop");
    ros::NodeHandle n;
    
    pub = n.advertise<modelica_bridge::ModComm>("modbridge_joy", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}