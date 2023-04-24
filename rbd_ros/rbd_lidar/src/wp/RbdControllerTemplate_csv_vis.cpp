#include "rbd_controller_template/RbdControllerTemplate.hpp"

// STD
#include <string>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// custom
#include <vector>

namespace rbd_controller_template {

RbdControllerTemplate::RbdControllerTemplate(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdControllerTemplate::topicCallback, this);
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



// functions to calculate critical angles from PointCloud2
float getAngleFromCol(int col){
  if((col>511) || (col<0)){   // Error handling
    ROS_ERROR("Horizontal index out of bounds. Must be in interval [0, 511]");
  }
  return 360*((float(col))/511);
}

std::vector<float> getCriticalAzimuths1(float* row){
  const int length = 512;
  std::vector<float> criticalAzimuths;
  
  for(int i = 0; i<length; i++){
    if(row[i]<1.0){
      criticalAzimuths.push_back(getAngleFromCol(i));
    }
  }
  return criticalAzimuths;
}

void printCriticalAzimuths(std::vector<float> criticalAzimuths){
  unsigned long int nr_crit = criticalAzimuths.size();
  printf("\nNumber of Critical Angles: %lu\n", nr_crit);
  printf("Critical Angles: (distance <1.0 m)\n");
  for(uint16_t i=0; i<criticalAzimuths.size(); i++){
    printf("%f deg\n", criticalAzimuths[i]);
  }
  printf("\n-------------------------------\n");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(inputPointCloud2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud);
  return pcl_cloud;
}


// main callback of this node
void RbdControllerTemplate::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  // convert PointCloud2& to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = convertToPCL(inputPointCloud2);

  // calculate squared range for each point and store it in 2D array
  int n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;
  float squared_range[n_rows][n_cols];

  std::string output = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/foo_";
  for(int row = 0; row <n_rows; row++){     // row
    float x,y,z;
    std::string file_path = output + std::to_string(row) + ".csv";
    std::ofstream myFile(file_path);

    myFile << "x, y, z\n";
    for(int col = 0; col<n_cols; col++){    // column
      int index_cloud = row*n_cols + col;

      x = pcl_cloud->points[index_cloud].x, y = pcl_cloud->points[index_cloud].y, z = pcl_cloud->points[index_cloud].z;
      squared_range[row][col] = x*x + y*y + z*z;
      myFile << x << "," << y << "," << z << std::endl;
    }
    myFile << std::endl;
    myFile.close();
  }
  

  // get critical Azimuths for row in lidar scan
  int rowToCheck = 0;
  std::vector<float> criticalAzimuths = getCriticalAzimuths1(squared_range[rowToCheck]);

  // print critical angles
  printCriticalAzimuths(criticalAzimuths);

  sleep(1);
}


bool RbdControllerTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "message";
  return true;
}

} /* namespace */
