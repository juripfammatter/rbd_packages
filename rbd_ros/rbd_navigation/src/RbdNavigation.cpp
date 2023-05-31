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
  tf::Pose tf_wiggle_1, tf_wiggle_2;

  try{
    tf::Quaternion q;
    q.setRPY(-0.3, 0.0, 0.0);
    tf_wiggle_1.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_wiggle_1.setRotation(q);

    q.setRPY(0.3, 0.0, 0.0);
    tf_wiggle_2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_wiggle_2.setRotation(q);

    toMsgPose(tf_wiggle_1, wiggle_pose_1);
    toMsgPose(tf_wiggle_2, wiggle_pose_2);
  } catch (tf::TransformException &exception){
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  /* Fill array (reverse order / pop_back) */
  wiggle_poses.push_back(zero_pose);
  wiggle_poses.push_back(wiggle_pose_2);
  wiggle_poses.push_back(zero_pose);
  wiggle_poses.push_back(wiggle_pose_1);
  


  /* Lie Down */
  tf::Pose tf_lie_down;

  try{
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    tf_lie_down.setOrigin(tf::Vector3(0.0, 0.0, -1.0));
    tf_lie_down.setRotation(q);

    toMsgPose(tf_lie_down, lie_down_pose);

  } catch (tf::TransformException &exception){
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  /* Fill array (reverse order / pop_back) */
  lie_down_poses.push_back(zero_pose);
  lie_down_poses.push_back(lie_down_pose);


  /* Walk */

  tf::Pose tf_walk_1;
  tf::Pose tf_walk_2;
  tf::Pose tf_walk_3;
  tf::Pose tf_walk_4;

  try{
    tf::Quaternion q;

    q.setRPY(0.0, 0.0, 0.0);
    tf_walk_1.setOrigin(tf::Vector3(0.6, 0.0, 0.0));
    tf_walk_1.setRotation(q);

    q.setRPY(0.0, 0.0, 3.14);
    tf_walk_2.setOrigin(tf::Vector3(-0.6, 0.0, 0.0));
    tf_walk_2.setRotation(q);

    q.setRPY(0.0, 0.0, 0.0);
    tf_walk_3.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_walk_3.setRotation(q);

    q.setRPY(0.0, 0.0, 0.0);
    tf_walk_4.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_walk_4.setRotation(q);

    toMsgPose(tf_walk_1, walk_pose_1);
    toMsgPose(tf_walk_2, walk_pose_2);
    toMsgPose(tf_walk_3, walk_pose_3);
    toMsgPose(tf_walk_4, walk_pose_4);

  } catch (tf::TransformException &exception){
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  /* Fill array (reverse order / pop_back) */
  walk_poses.push_back(walk_pose_2);
  walk_poses.push_back(walk_pose_1);



  /* Spin */
  tf::Pose tf_spin_1;
  tf::Pose tf_spin_2;

  try{
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 3.14);
    tf_spin_1.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_spin_1.setRotation(q);

    q.setRPY(0.0, 0.0, 0.0);
    tf_spin_2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_spin_2.setRotation(q);

    toMsgPose(tf_spin_1, spin_pose_1);
    toMsgPose(tf_spin_2, spin_pose_2);

  } catch (tf::TransformException &exception){
    ROS_WARN("%s", exception.what());
    ros::Duration(1.0).sleep();
  }

  /* Fill array (reverse order / pop_back) */
  spin_poses.push_back(spin_pose_2);
  spin_poses.push_back(spin_pose_1);


}



bool RbdNavigation::serviceCallback(rbd_msgs::GeneratePath::Request& request, 
                                    rbd_msgs::GeneratePath::Response& response)
{
  if(request.command == "wiggle"){
    response.poses = wiggle_poses;
    response.nr_of_poses = 4;
    response.namedPoses = {"zero position", "bow up", "zero position", "bow down"}; // reverse order since they will be dequed with pop_back
    return true;

  } else if(request.command == "walk"){
    response.poses = walk_poses;
    response.nr_of_poses = 2;
    response.namedPoses = { "walk home", "walk forward"};
    return true;

  } else if(request.command == "spin"){
    response.poses = spin_poses;
    response.nr_of_poses = 2;
    response.namedPoses = {"rotate back", "rotate 180"};
    return true;

  }else if(request.command == "lie_down"){
    response.poses = lie_down_poses;
    response.nr_of_poses = 2;
    response.namedPoses = {"stand up", "lie down"};
    return true;
    
  }
  return false;
}

} /* namespace */
