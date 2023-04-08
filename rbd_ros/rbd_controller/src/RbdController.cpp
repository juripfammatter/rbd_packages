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

  //Create Service Servers / Clients
  // serviceServer_ = nodeHandle_.advertiseService("get_average",&RbdController::serviceCallback, this);
  navigationClient_ = nodeHandle_.serviceClient<rbd_msgs::GeneratePath>(path_generation_service);
  executionClient_ = nodeHandle_.serviceClient<rbd_msgs::SetPosition>(set_postion_service);
  modeClient_ = nodeHandle_.serviceClient<qre_msgs::RbdMode>(set_mode_service);

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
      !nodeHandle_.getParam("set_mode_service", set_mode_service)) return false;
  return true;
}



void RbdController::statusCallback(const std_msgs::String& message)
{
  // main Functions are implemented here

  /* Gesture */
  if (gesture_needed){
    //call gesture Service
    //TBD command based on gestrec
    command = "wiggle";

    gesture_needed = false;
    path_needed = true;
  }

  /* Path */
  if (path_needed){
    //call GeneratePath service

    navigation_srv.request.command = command;
    ROS_INFO_STREAM("creating navigation request with command: "<<command);
    if (navigationClient_.call(navigation_srv))
    {
     // handle response
      nr_of_poses = navigation_srv.response.nr_of_poses;
      poses = navigation_srv.response.poses;
      ROS_INFO_STREAM("Successfully generated path with "<< nr_of_poses<<" poses");

    } else {
      ROS_ERROR("Failed to call service generate_path");
    }

    path_needed = false;
  }

  /* Execution*/
  std::string execution_status = message.data;

  if(execution_status != "em_stop"){
    if(execution_status == "idle"){

      //dequeue Pose and remove
      current_pose = poses.back();
      poses.pop_back();

      //call SetPosition Service with current Pose
      execution_srv.request.goal = current_pose;

      if (executionClient_.call(execution_srv))
      {
        // handle response
        ROS_INFO_STREAM(""<< execution_srv.response.message);

      } else {
        ROS_ERROR("Failed to call service set_position");
      }

      //check if vector is empty
      if (poses.empty()){
        gesture_needed = true;

        //set mode to walk
        mode_srv.request.mode = 2;
        if (modeClient_.call(mode_srv))
        {
          // handle response
          ROS_INFO_STREAM("Mode set to walk Succesfully");

        } else {
          ROS_ERROR("Failed to call service set_mode");
        }

        ros::WallDuration(1.0).sleep(); 

        //set mode back to stand
        mode_srv.request.mode = 1;
        if (modeClient_.call(mode_srv))
        {
          // handle response
          ROS_INFO_STREAM("Mode set to stand succesfully");

        } else {
          ROS_ERROR("Failed to call service set_mode");
        }
      }
    }
  } 
  else 
  {
    // emergency behaviour
  }
}



// bool RbdController::serviceCallback(std_srvs::Trigger::Request& request,
//                                          std_srvs::Trigger::Response& response)
// {
//   response.success = true;
//   response.message = "message";
//   return true;
// }

} /* namespace */