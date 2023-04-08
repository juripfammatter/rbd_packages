#pragma once

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <unitree_legged_msgs/HighState.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>

namespace rbd_localization {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RbdLocalization
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdLocalization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RbdLocalization();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);

  void updateState(const unitree_legged_msgs::HighState& state);

  float lowPassFilter(const float u, float& u_old, float Ts);

  void imuIntegrate(const unitree_legged_msgs::HighState& state, tf::Pose& pose);

  /*!
   * ROS topic callback method.
   */
  void stateCallback(const unitree_legged_msgs::HighState& state);
  void imuZedCallback(const sensor_msgs::Imu& imu);

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
  ros::Subscriber statesubscriber_;
  ros::Subscriber imuZedSubscriber_;


  ros::Publisher positionPublisher_;
  ros::Publisher imuQrePublisher_;
  ros::Publisher imuZedPublisher_;
  geometry_msgs::Pose position_message;
  sensor_msgs::Imu imu_qre_message;
  sensor_msgs::Imu imu_zed_message;

  //! ROS topic name to subscribe to.
  std::string stateSubscriberTopic_;
  std::string imuZedSubscriberTopic_;

  std::string positionPublisherTopic_;
  std::string imuQrePublisherTopic_;
  std::string imuZedPublisherTopic_;

  //! ROS service server.
  // ros::ServiceServer serviceServer_;

  //Time
  ros::WallTime t_old_qre;
  ros::WallTime t_old_zed;

  // Imu data
  //TBD tf
  std::vector<float> imu_acc = std::vector<float> (3);
  std::vector<float> imu_gyr = std::vector<float> (3);
  std::vector<float> imu_quat = std::vector<float> (4);
  float imu_temp;

  // Position
  float x_old, y_old, z_old;

  // Imu filter
  float imu_qre_filter_x_old = 0;
  float imu_qre_filter_y_old = 0;
  float imu_qre_filter_z_old = 0;

  float imu_zed_filter_x_old = 0;
  float imu_zed_filter_y_old = 0;
  float imu_zed_filter_z_old = 0;

};

} /* namespace */
