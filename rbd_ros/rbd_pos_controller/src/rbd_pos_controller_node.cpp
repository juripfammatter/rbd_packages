#include <ros/ros.h>
#include "rbd_pos_controller/RbdPosController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbd_pos_controller");
  ros::NodeHandle nodeHandle("~");

  rbd_pos_controller::RbdPosController rbdPosController(nodeHandle);

  ros::spin();
  return 0;
}
