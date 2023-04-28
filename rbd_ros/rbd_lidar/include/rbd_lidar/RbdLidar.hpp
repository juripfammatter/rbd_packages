#pragma once

//! ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <stdint.h>
#include <std_srvs/SetBool.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//! STD
#include <string>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <bits/stdc++.h>
#include <math.h>

namespace rbd_lidar {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RbdLidar
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdLidar(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RbdLidar();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const sensor_msgs::PointCloud2& message);

  //! Private functions
  float getAzimuthDegFromCol(uint32_t col);
  std::vector<float> getCriticalAzimuthsDeg(uint32_t* row, uint32_t rowindex);
  void printList(std::list<std::vector<float> >& listOfVectors);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! loaded from default.yaml
  std::string subscriberTopic_;
  int critical_distance_sect1_sect3_mm_load;
  int critical_distance_sect2_sect4_mm_load;

  //! ROS client for collision avoidance service
  ros::ServiceClient collisionClient;
  std_srvs::SetBool collisionSrv;

  //! Private variables
  // thresholds and countersfor collision detection
  uint32_t critical_distance_sect1_sect3_mm = 1000;   // default. Overwritten by critical_distance_sect1_sect3_mm_load
  uint32_t critical_distance_sect2_sect4_mm = 1000;   // default. Overwritten by critical_distance_sect2_sect4_mm_load
  uint32_t blind_zone = 250;                          // lidar cannot measure closer
  uint32_t threshold_crit_azimuths = 20;
  uint32_t nr_crit_azimuths = 0;                      // counter

  // collision state
  bool collision, last_collision;

  // parameters for collision detection sections
  uint32_t l_dog = 500, w_dog = 300, h_dog =400;      // dog dimensions (standing, according to datasheet)
  float alpha, beta, phi1, phi2, phi3, phi4;          // angles for collision detection sections

  // parameters for point cloud
  uint32_t n_rows = 0, n_cols = 0;                    // default. Are loaded when running

  // auxiliary variables
  double deg_to_rad_factor = M_PI/180.0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
  uint32_t row_pcl = 0;
  ros::Publisher pub_PC2;
};

} /* namespace */
