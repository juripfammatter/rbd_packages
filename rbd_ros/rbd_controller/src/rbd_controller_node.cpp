#include <ros/ros.h>
#include "rbd_controller/RbdController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbd_controller");
  ros::NodeHandle nodeHandle("~");

  rbd_controller::RbdController rbdController(nodeHandle);

  ros::spin();
  return 0;
}
