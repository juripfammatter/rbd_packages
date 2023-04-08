#include "rbd_navigation/RbdNavigation.hpp"

// STD
#include <string>

namespace rbd_navigation {

RbdNavigation::RbdNavigation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  initChoreography();

  // // Create subscribers
  // subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdNavigation::topicCallback, this);

  // Create service servers/clients
  serviceServer_ = nodeHandle_.advertiseService(generate_path_service,&RbdNavigation::serviceCallback, this);

  ROS_INFO("Successfully launched node.");
}

RbdNavigation::~RbdNavigation()
{
}



bool RbdNavigation::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_) ||
      !nodeHandle_.getParam("generate_path_service", generate_path_service)) return false;
  return true;
}

// Pass by reference!!
void RbdNavigation::toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose)
  {
      msg_pose.position.x = tf_pose.getOrigin().getX();
      msg_pose.position.y = tf_pose.getOrigin().getY();
      msg_pose.position.z = tf_pose.getOrigin().getZ();

      tf::Quaternion orientation = tf_pose.getRotation();
      msg_pose.orientation.w = orientation.getW();
      msg_pose.orientation.x = orientation.getX();
      msg_pose.orientation.y = orientation.getY();
      msg_pose.orientation.z = orientation.getZ();
  }

void RbdNavigation::initChoreography(void){
  
  /*Wiggle */
  tf::Pose tf_wiggle_left_pose, tf_wiggle_right_pose;

  try{
    tf::Quaternion q;
    q.setRPY(0.0,0.2,0.0);
    tf_wiggle_left_pose.setOrigin(tf::Vector3(0,0,0));
    tf_wiggle_left_pose.setRotation(q);

    q.setRPY(0.0,-0.2,0.0);
    tf_wiggle_right_pose.setOrigin(tf::Vector3(0,0,0));
    tf_wiggle_right_pose.setRotation(q);

    toMsgPose(tf_wiggle_left_pose, wiggle_left_pose);
    toMsgPose(tf_wiggle_right_pose, wiggle_right_pose);
  } catch (tf::TransformException &exception){
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  wiggle_poses.push_back(zero_pose);
  wiggle_poses.push_back(wiggle_left_pose);
  wiggle_poses.push_back(wiggle_right_pose);
  wiggle_poses.push_back(wiggle_left_pose);
  wiggle_poses.push_back(wiggle_right_pose);
  wiggle_poses.push_back(wiggle_left_pose);
  wiggle_poses.push_back(wiggle_right_pose);
  wiggle_poses.push_back(zero_pose);
}

// void RbdNavigation::topicCallback(const sensor_msgs::Temperature& message)
// {
//   // main Functions are implemented here
// }



bool RbdNavigation::serviceCallback(rbd_msgs::GeneratePath::Request& request, 
                                    rbd_msgs::GeneratePath::Response& response)
{
  if(request.command == "wiggle"){
    response.poses = wiggle_poses;
    response.nr_of_poses = 8;
    return true;
  }
    return false;
}

} /* namespace */
