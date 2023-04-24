#include "rbd_controller_template/RbdControllerTemplate.hpp"



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
float RbdControllerTemplate::getAngleFromCol(int col){
  if((col>511) || (col<0)){   // Error handling
    ROS_ERROR("Horizontal index out of bounds. Must be in interval [0, 511]");
  }
  return 360*((float(col))/511);
}

std::vector<float> RbdControllerTemplate::getCriticalAzimuths1(float* row){
  const int length = 512;
  std::vector<float> criticalAzimuths;
  
  for(int i = 0; i<length; i++){
    if(row[i]<1.0){
      criticalAzimuths.push_back(getAngleFromCol(i));
      n_crit++;
    }
  }
  return criticalAzimuths;
}

void RbdControllerTemplate::printCriticalAzimuths(std::vector<float> criticalAzimuths){
  unsigned long int n_crit_row = criticalAzimuths.size();
  printf("\nNumber of Critical Angles: %lu\n", n_crit_row);
  printf("Critical Angles: (distance <1.0 m)\n");
  for(uint16_t i=0; i<criticalAzimuths.size(); i++){
    printf("%f deg\n", criticalAzimuths[i]);
  }
  printf("\n-------------------------------\n");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RbdControllerTemplate::convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(inputPointCloud2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud);
  return pcl_cloud;
}


// main callback of this node
void RbdControllerTemplate::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  // counter for critical angles
  n_crit = 0;

  // convert PointCloud2& to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = convertToPCL(inputPointCloud2);

  // init file to write coordinates of last message to pcl_list.csv
  std::string file_path = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/pcl_list.csv";
  std::ofstream myFile(file_path);
  myFile << "x, y, z\n";

  // calculate squared range for each point and store it in 2D array
  int n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;
  float squared_range[n_rows][n_cols];

  for(int row = 0; row <n_rows; row++){     // row
    float x,y,z;
    for(int col = 0; col<n_cols; col++){    // column
      int index_cloud = row*n_cols + col;
      x = pcl_cloud->points[index_cloud].x, y = pcl_cloud->points[index_cloud].y, z = pcl_cloud->points[index_cloud].z;
      squared_range[row][col] = x*x + y*y + z*z;
      myFile << x << "," << y << "," << z << std::endl;
    }
  }
  myFile.close();


  // get critical Azimuths for row in lidar scan
  std::vector<float> criticalAzimuths;
  for(int rowToCheck = 0; rowToCheck<n_rows; rowToCheck++){
    // note: only the last row is conserved, but the counter for critical angles is increased for all rows
    criticalAzimuths = getCriticalAzimuths1(squared_range[rowToCheck]);
  }

  // print critical angles
  printCriticalAzimuths(criticalAzimuths);
  printf("Total number of critical azimuths: %d\n", n_crit);
  sleep(1);
}

bool RbdControllerTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  if(n_crit>20){
    response.message = "object_approaching";
  }
  else{
    response.message = "no_objects";
  }
  return true;
}

} /* namespace */
