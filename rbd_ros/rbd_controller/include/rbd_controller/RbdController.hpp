#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <rbd_msgs/GeneratePath.h>
#include <rbd_msgs/SetPosition.h>
#include <qre_msgs/RbdMode.h>

namespace rbd_controller {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RbdController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RbdController();

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
  void statusCallback(const std_msgs::String& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  // bool serviceCallback(std_srvs::Trigger::Request& request,
  //                      std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber statusSubscriber_;

  //! ROS topic and service  names from parameter server.
  std::string statusSubscriberTopic_;
  std::string path_generation_service;
  std::string set_postion_service;
  std::string set_mode_service;

  //! ROS service server.
  // ros::ServiceServer serviceServer_;
  ros::ServiceClient navigationClient_;
  ros::ServiceClient executionClient_;
  ros::ServiceClient modeClient_;

  rbd_msgs::GeneratePath navigation_srv;
  rbd_msgs::SetPosition execution_srv;
  qre_msgs::RbdMode mode_srv;

  std::vector<geometry_msgs::Pose> poses;
  geometry_msgs::Pose current_pose;
  int32_t nr_of_poses;

  bool gesture_needed = true;
  bool path_needed = false;

  std::string command;

};

} /* namespace */
