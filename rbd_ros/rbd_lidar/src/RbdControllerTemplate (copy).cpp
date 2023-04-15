#include "rbd_controller_template/RbdControllerTemplate.hpp"

// STD
#include <string>

namespace rbd_controller_template {

RbdControllerTemplate::RbdControllerTemplate(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  //*
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdControllerTemplate::topicCallback, this);
  //*/
  //subscriber_ = nodeHandle_.subscribe("/ouster/points", 1,&RbdControllerTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",&RbdControllerTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");


}

RbdControllerTemplate::~RbdControllerTemplate()
{
}



bool RbdControllerTemplate::readParameters()
{
  // get "subscriber_topic: /ouster/points" from .yaml file and save it into subscriberTopic_ variable
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;    
  return true;
}

void RbdControllerTemplate::topicCallback(const sensor_msgs::PointCloud2& pCloud)
{

  // get width and height of 2D point cloud data
  uint32_t width = pCloud.width;           // 512, 1024, 2048
  uint32_t height = pCloud.height;         // 32 for our Ouster OS1
  if(width == 0 || height == 0){
    ROS_WARN("witdh or height is equal to zero");
  }

  ROS_INFO("width: %d", width);
  // iterate horizontally
  //uint8_t u = 0;
  //uint32_t v = 0;
  uint32_t currcol = 0;
  // rows
  for(uint32_t j = 0; j<5; j++){
    ROS_INFO("Row nr. %d", j);
    // cols
    for(uint32_t i = 0; i<width; i+=100){   
      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition =  currcol*pCloud.point_step + j*pCloud.row_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

      ROS_INFO("Row %d, Point nr. %d, X0, Y0, Z0: %f, %f, %f", j, i, X, Y, Z);
      currcol++;

    }
    ROS_INFO("End of row %d\n\n", j);
  }
  ROS_INFO("\n");




  sleep(10);
}






bool RbdControllerTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "message";
  return true;
}

} /* namespace */
