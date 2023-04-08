#include <ros/ros.h>
#include "rbd_localization/RbdLocalization.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbd_localization");
  ros::NodeHandle nodeHandle("~");

  rbd_localization::RbdLocalization RbdLocalization(nodeHandle);

  ros::spin();
  return 0;
}
