#include <ros/ros.h>
#include "rbd_controller_template/RbdControllerTemplate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbd_controller_template");
  ros::NodeHandle nodeHandle("~");

  rbd_controller_template::RbdControllerTemplate rbdControllerTemplate(nodeHandle);

  ros::spin();
  return 0;
}
