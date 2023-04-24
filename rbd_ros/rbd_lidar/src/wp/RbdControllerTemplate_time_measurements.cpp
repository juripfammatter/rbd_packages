#include "rbd_controller_template/RbdControllerTemplate.hpp"

// STD
#include <string>
#include <stdio.h>
#include <chrono>
#include <iostream>

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
  printf("\nNumber of Critical Angles: %lu\n", criticalAzimuths.size());
  printf("Critical Angles: (distance <1.0 m)\n");
  for(uint16_t i=0; i<criticalAzimuths.size(); i++){
    printf("%f deg\n", criticalAzimuths[i]);
  }
  printf("\n-------------------------------\n");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(inputPointCloud2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  return temp_cloud;
}


// main callback of this node
void RbdControllerTemplate::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  using namespace std::chrono;
  auto start_Callback = high_resolution_clock::now();
  static int i = 0;
  i++;
  printf("Received PointCloud2  nr. %d\n", i);
  printf("width: %d, heigth: %d\n", inputPointCloud2.width, inputPointCloud2.height);

  // convert PointCloud2& to pcl
  auto start = high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = convertToPCL(inputPointCloud2);
  auto stop = high_resolution_clock::now();  auto duration = duration_cast<microseconds>(stop - start);
  std::cout << "Duration_PC2_to_pcl: " << duration.count() << " us" << std::endl;


  // calculate squared range for each point and store it in 2D array
  start = high_resolution_clock::now();

  int n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;
  float squared_range[n_rows][n_cols];
  for(int row = 0; row <n_rows; row++){
    for(int col = 0; col<n_cols; col++){
      int index_cloud = row*n_cols + col;

      float x = temp_cloud->points[index_cloud].x, y = temp_cloud->points[index_cloud].y, z = temp_cloud->points[index_cloud].z;
      squared_range[row][col] = x*x + y*y + z*z;
    }
  }

  stop = high_resolution_clock::now();  duration = duration_cast<microseconds>(stop - start);
  std::cout << "Duration_pcl_to_2D: " << duration.count() << " us" << std::endl;

  // get critical Azimuths for row in lidar scan
  int rowToCheck = 0;
  start = high_resolution_clock::now();
  std::vector<float> criticalAzimuths = getCriticalAzimuths1(squared_range[rowToCheck]);
  stop = high_resolution_clock::now();  duration = duration_cast<microseconds>(stop - start);
  std::cout << "Duration_get_Azimuths: " << duration.count() << " us" << std::endl;

  // print critical angles
  //printCriticalAzimuths(criticalAzimuths);
  puts("Calculated criticalAzimuths");
  auto stop_Callback = high_resolution_clock::now();  duration = duration_cast<microseconds>(stop_Callback - start_Callback);
  std::cout << "-----Time for whole function-----: " << duration.count() << " us\n" << std::endl;

  //sleep(15);
}


bool RbdControllerTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "message";
  return true;
}

} /* namespace */
