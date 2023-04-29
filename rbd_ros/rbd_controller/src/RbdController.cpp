#include "rbd_controller/RbdController.hpp"

// STD
#include <string>

namespace rbd_controller {

RbdController::RbdController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Create Subscribers
  statusSubscriber_ = nodeHandle_.subscribe(statusSubscriberTopic_, 1,&RbdController::statusCallback, this);
  
  // Create Publisher
  namedPosePublisher_ = nodeHandle_.advertise<rbd_msgs::NamedPoses>(named_pose_topic,1);

  //Create Service Servers / Clients
  navigationClient_ = nodeHandle_.serviceClient<rbd_msgs::GeneratePath>(path_generation_service);
  executionClient_ = nodeHandle_.serviceClient<rbd_msgs::SetPosition>(set_postion_service);
  modeClient_ = nodeHandle_.serviceClient<qre_msgs::RbdMode>(set_mode_service);
  gestureClient_ = nodeHandle_.serviceClient<rbd_msgs::GetLastGesture>(last_gesture_service);

  ROS_INFO("Successfully launched node.");
}

RbdController::~RbdController()
{
}



bool RbdController::readParameters()
{
  if (!nodeHandle_.getParam("status_subscriber_topic", statusSubscriberTopic_) ||
      !nodeHandle_.getParam("path_generation_service", path_generation_service) ||
      !nodeHandle_.getParam("set_postion_service", set_postion_service) ||
      !nodeHandle_.getParam("set_mode_service", set_mode_service) ||
      !nodeHandle_.getParam("last_gesture_service", last_gesture_service) ||
      !nodeHandle_.getParam("named_pose_topic", named_pose_topic)) return false;
  return true;
}



void RbdController::statusCallback(const std_msgs::String& message)
{
  execution_status = message.data;

  /*************************** State machine ****************************/
  /* 1. Wait for gesture 
  *  2. Generate Path
  *  3. Execute Pose
  */

  if(execution_status == "em_stop"){
    ROS_INFO_STREAM_ONCE("em_stop enabled");

  }else if(execution_status == "collision"){
    ROS_INFO_STREAM_ONCE("collision detected");
    
  }else if(execution_status == "idle"){
    switch(control_state){
      case WAITING_FOR_GESTURE:
          /* Attentive Pose*/
          attentive_pose.orientation.w = 0.9848078;
          attentive_pose.orientation.y = -0.1736482;
          execution_srv.request.goal =  attentive_pose;

          if (executionClient_.call(execution_srv)){
            ROS_INFO_STREAM(""<< execution_srv.response.message);
          } else {
            ROS_ERROR("Failed to call service set_position with attentive position");
          }

          /* Request Gesture */
          if (!gestureClient_.call(gesture_srv)){
            ROS_ERROR("Failed to call service get_last_gesture");
          } else {
            command = gesture_srv.response.last_gesture;
            ROS_INFO_STREAM("received gesture: " << command);

            /* Zero Pose */
            execution_srv.request.goal =  zero_pose;
            
            if (executionClient_.call(execution_srv)){
              ROS_INFO_STREAM(""<< execution_srv.response.message);
              control_state = GENERATE_PATH;
            } else {
              ROS_ERROR("Failed to call service set_position with zero position");
            }
          }
        break;


      case GENERATE_PATH:
          /* Get Path*/
          navigation_srv.request.command = command;
          ROS_INFO_STREAM("creating navigation request with command: "<<command);

          if (navigationClient_.call(navigation_srv)){
            nr_of_poses = navigation_srv.response.nr_of_poses;
            poses = navigation_srv.response.poses;
            ROS_INFO_STREAM("Successfully generated path with "<< nr_of_poses<<" poses");

            /* named poses */
            named_poses_array = navigation_srv.response.namedPoses;

            control_state = POSE_EXECUTION;
          } else {
            ROS_ERROR("Failed to call service generate_path or mismatching command");
          }
        break;


      case POSE_EXECUTION:
          /* dequeue Pose and remove */
          current_pose = poses.back();
          poses.pop_back();

          /* publish to named poses topic for visualization*/
          named_poses.namedPoses = named_poses_array;
          namedPosePublisher_.publish(named_poses);
          named_poses_array.pop_back();

          /* Set pose*/
          execution_srv.request.goal = current_pose;

          if (executionClient_.call(execution_srv)){
            ROS_INFO_STREAM(""<< execution_srv.response.message);
            ros::WallDuration(0.1).sleep();

          } else {
            ROS_ERROR("Failed to call service set_position");
          }

          /* Change mode for a short amount of time to reset foot position*/
          if (poses.empty()){
            /* set mode to walk */
            // mode_srv.request.mode = 2;
            // if (modeClient_.call(mode_srv)){
            //   ROS_INFO_STREAM("Mode set to walk Succesfully");
            // } else {
            //   ROS_ERROR("Failed to call service set_mode with mode 2");
            // }

            // ros::WallDuration(0.5).sleep(); 

            // /* set mode back to stand */
            // mode_srv.request.mode = 1;
            // if (modeClient_.call(mode_srv)){
            //   ROS_INFO_STREAM("Mode set to stand succesfully");
            //   control_state = WAITING_FOR_GESTURE;
            // } else {
            //   ROS_ERROR("Failed to call service set_mode with mode 1");
            // }
            named_poses.namedPoses = named_poses_array;
            namedPosePublisher_.publish(named_poses);
            control_state = WAITING_FOR_GESTURE;
          }
        break;
    }
  } else {
    ROS_INFO_STREAM_THROTTLE(0.5,"main controller running");
  }
}

} /* namespace */
