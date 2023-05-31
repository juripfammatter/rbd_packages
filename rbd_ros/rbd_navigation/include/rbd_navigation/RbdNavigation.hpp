#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <rbd_msgs/GeneratePath.h>
#include <tf/tf.h>
#include <vector>


namespace rbd_navigation {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RbdNavigation
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RbdNavigation(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor. 
   */
  virtual ~RbdNavigation();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * @brief converts tf pose to geometry_msgs pose (pass by reference!)
   * @param tf_pose pose to be converted
   * @param msg_pose resulting pose in geometry_msgs type
   */
  void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);

  /*!
   * @brief initializes the _poses array (currently only wiggle_poses)
   * @param  
   */
  void initChoreography(void);


  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(rbd_msgs::GeneratePath::Request& request,
                       rbd_msgs::GeneratePath::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic and service names to subscribe to.
  std::string subscriberTopic_;
  std::string generate_path_service;

  //! ROS service server.
  ros::ServiceServer serviceServer_;


  //Custom poses
  std::vector<geometry_msgs::Pose> wiggle_poses;
  std::vector<geometry_msgs::Pose> walk_poses;
  std::vector<geometry_msgs::Pose> spin_poses;
  std::vector<geometry_msgs::Pose> lie_down_poses;
  std::vector<geometry_msgs::Pose> sit_poses;


  geometry_msgs::Pose zero_pose;
  geometry_msgs::Pose wiggle_pose_1; 
  geometry_msgs::Pose wiggle_pose_2;

  geometry_msgs::Pose lie_down_pose; 


  geometry_msgs::Pose walk_pose_1;
  geometry_msgs::Pose walk_pose_2;
  geometry_msgs::Pose walk_pose_3;
  geometry_msgs::Pose walk_pose_4;

  geometry_msgs::Pose spin_pose_1;
  geometry_msgs::Pose spin_pose_2;

};

} /* namespace */
