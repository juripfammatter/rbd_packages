#pragma once

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <rbd_msgs/SetPosition.h>
#include <tf/tf.h>
#include <qre_msgs/SetBodyPose.h>
#include <cmath>

namespace rbd_pos_controller {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RbdPosController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdPosController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RbdPosController();


 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  float saturate(double value, double lower_limit, double upper_limit);
  void updateGain(void);
  void envelope(double &vx, double &vy);

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void actualPositionCallback(const geometry_msgs::Pose& pose);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool emStopCallback(std_srvs::SetBool::Request& request,
                       std_srvs::SetBool::Response& response);

  bool setPosCallback(rbd_msgs::SetPosition::Request& request,
                       rbd_msgs::SetPosition::Response& response);

                    
  //
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber subscriber_;
  ros::Publisher cmdVelPublisher_;
  ros::Publisher statusPublisher_;
  ros::ServiceServer emServiceServer_;
  ros::ServiceServer posServiceServer_;
  ros::ServiceClient poseClient_;
  qre_msgs::SetBodyPose pose_srv;

  std::string sub_topic;
  std::string cmd_vel_pub_topic;
  std::string status_pub_topic;
  std::string body_pose_service;
  std::string emergency_stop_service;
  std::string set_position_service;
  std_msgs::String status_message;
  geometry_msgs::Twist stop_message;
  geometry_msgs::Twist vel_message;
  geometry_msgs::Twist zero_twist;
  geometry_msgs::Point goal_position;
  geometry_msgs::Point actual_position;
  geometry_msgs::Quaternion actual_orientation;

  double goal_roll = 0, goal_pitch = 0, goal_yaw = 0;
  double remaining_distance;

  bool emergency_stop = true;
  
  double kp_x, kp_y;
  double k_roll = 2.865, k_pitch = 2.865, k_yaw=2.046;
  double zero_height = 0.41;
  double ctrl_vel_x, ctrl_vel_y;
};

} /* namespace */
