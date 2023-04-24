#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <stdint.h>
#include <std_srvs/SetBool.h>

// STD
#include <string>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <bits/stdc++.h>


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

  // Private functions
  float getAzimuthFromCol(uint32_t col);
  std::vector<float> getCriticalAzimuths(uint32_t* row);
  void printList(std::list<std::vector<float> >& listOfVectors);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! loaded from default.yaml
  std::string crit_dist_str_;

  //! ROS client for collision avoidance service
  ros::ServiceClient collisionClient;
  std_srvs::SetBool collisionSrv;


  //! Private variables
  uint32_t critical_distance_mm = 1000;
  uint32_t nr_crit_azimuths = 0;
  uint32_t threshold_crit_azimuths = 20;
  uint32_t n_rows = 0, n_cols = 0;


};

} /* namespace */
