#include <ros/ros.h>
#include "rbd_lidar/RbdLidar.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbd_lidar");
  ros::NodeHandle nodeHandle("~");

  rbd_lidar::RbdLidar RbdLidar(nodeHandle);

  ros::spin();
  return 0;
}
