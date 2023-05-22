#pragma once

//! ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <stdint.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

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
  void statusTopicCallback(const std_msgs::Bool& message);

  //! Private functions
  
  /** surveys critical zone and finds critical azimuths.
   *  measures, whether a point is inside the specified critical distances (e. g. "critical_distance_back_mm").
   *  marks critical points by color
   *  counts number of critical points and returns a vector with the critical azimuths (horizontal angles) 
   *
   * @param param1 one row (in total 32 per scan) of ranges in a 360° lidar scan
   * @return Vector of all critical azimuths in "row", as float values
   */
  std::vector<float> findCollisions_getCritAzimuths(uint32_t* row);
  /** prints each vector from a list of vectors
   */
  void printList(std::list<std::vector<float> >& listOfVectors);
  /** converts "PointCloud2" (ROS) to "pcl::PointXYZI" (C++)
   *  takes a reference to a sensor_msgs::PointCloud2 message and converts it to pcl::PointXYZI
   *
   * @param param1 sensor_msgs::PointCloud2 message. A full 360° scan from the lidar (x, y, z, intensity, reflectivity, metadata etc. )
   * @return pcl::PointXYZI point cloud (C++ type: only  x, y, z, intensity)
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;
  ros::Subscriber subscriber_status;


  //! ROS client for collision avoidance service
  ros::ServiceClient collisionClient;
  std_srvs::SetBool collisionSrv;
  

  //! loaded from default.yaml
  std::string subscriberTopic_;
  std::string slam_status_subscriber_topic;
  std::string collision_service;
  std::string flagged_pointcloud;
  std::string scan_pointcloud;
  int critical_distance_back_mm_load;
  int critical_distance_left_mm_load;
  int critical_distance_front_mm_load;
  int critical_distance_right_mm_load;
  int lim_rows_back_load;

  //! Private variables
  // thresholds and counters for collision detection
  float critical_distance_back_mm = 1000;                          // default. Overwritten by value from 'default.yaml'
  float critical_distance_left_mm = 1000;
  float critical_distance_front_mm = 1000;
  float critical_distance_right_mm = 1000;
  uint32_t lim_rows_back = 32;

  uint32_t blind_zone = 250;                                        // lidar cannot measure closer
  uint32_t threshold_crit_azimuths = 70;
  uint32_t nr_crit_azimuths = 0;                                    // counter

  // collision state
  bool collision, last_collision;

  // azimuths for collision detection sections
  float alpha_deg, beta_deg, gamma_deg, delta_deg, phi1_deg, phi2_deg, phi3_deg, phi4_deg;          // angles for collision detection sections

  // point cloud
  uint32_t n_rows = 0, n_cols = 0;                                  // default. Are loaded when running
  uint32_t row_pcl = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_scan;
  ros::Publisher pub_PC2;
  ros::Publisher pub_PC2_scan;

  // SLAM
  bool rbd_is_running = false;

  // auxiliary variables
  double deg_to_rad_factor = M_PI/180.0;
  double rad_to_deg_factor = 180.0/M_PI;

};

} /* namespace */
