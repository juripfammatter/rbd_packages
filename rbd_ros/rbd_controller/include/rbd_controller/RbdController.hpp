#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <rbd_msgs/GeneratePath.h>
#include <rbd_msgs/SetPosition.h>
#include <qre_msgs/RbdMode.h>
#include <qre_msgs/SetBodyPose.h>
#include <rbd_msgs/GetLastGesture.h>

namespace rbd_controller {

/**
 * Main class for the node to handle the ROS interfacing.
 */
class RbdController
{
 public:
  /**
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdController(ros::NodeHandle& nodeHandle);

  /**
   * Destructor.
   */
  virtual ~RbdController();

 private:
  /**
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /**
   * ROS topic callback method.
   * @param message the received message.
   */
  void statusCallback(const std_msgs::String& message);

  /* ROS node handle */
  ros::NodeHandle& nodeHandle_;

  /* ROS topic subscriber */
  ros::Subscriber statusSubscriber_;

  /* ROS topic and service names from parameter server */
  std::string statusSubscriberTopic_;

  std::string path_generation_service;
  std::string set_postion_service;
  std::string set_mode_service;
  std::string last_gesture_service;

  /* ROS service clients */
  ros::ServiceClient navigationClient_;
  ros::ServiceClient executionClient_;
  ros::ServiceClient modeClient_;
  ros::ServiceClient gestureClient_;

  /* ROS service messages */
  rbd_msgs::GeneratePath navigation_srv;
  rbd_msgs::SetPosition execution_srv;
  rbd_msgs::GetLastGesture gesture_srv;
  qre_msgs::RbdMode mode_srv;
  

  /* global variables */
  std::vector<geometry_msgs::Pose> poses;
  geometry_msgs::Pose zero_pose;
  geometry_msgs::Pose attentive_pose;
  geometry_msgs::Pose current_pose;
  int32_t nr_of_poses;

  std::string execution_status = "em_stop";
  std::string command;


  /* Control state for state machine */
  typedef enum{
      WAITING_FOR_GESTURE,
      GENERATE_PATH,
      POSE_EXECUTION
  }control_state_t;
  control_state_t control_state;

};

} /* namespace */
