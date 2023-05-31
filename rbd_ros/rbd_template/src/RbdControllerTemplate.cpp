#include "rbd_controller_template/RbdControllerTemplate.hpp"

// STD
#include <string>

namespace rbd_controller_template {

RbdControllerTemplate::RbdControllerTemplate(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdControllerTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",&RbdControllerTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RbdControllerTemplate::~RbdControllerTemplate()
{
}



bool RbdControllerTemplate::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}



void RbdControllerTemplate::topicCallback(const sensor_msgs::Temperature& message)
{
  // main Functions are implemented here
}



bool RbdControllerTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "message";
  return true;
}

} /* namespace */
