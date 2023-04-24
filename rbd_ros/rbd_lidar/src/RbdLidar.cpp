#include "rbd_lidar/RbdLidar.hpp"



namespace rbd_lidar {

RbdLidar::RbdLidar(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdLidar::topicCallback, this);
  ROS_INFO("Loaded critical distance: %s mm", crit_dist_str_.c_str());
  ROS_INFO("Successfully launched node.");
  collisionClient = nodeHandle_.serviceClient<std_srvs::SetBool>("/collision");

}

RbdLidar::~RbdLidar()
{
}

bool RbdLidar::readParameters()
{
  // get "subscriber_topic: /ouster/points" from .yaml file and save it into subscriberTopic_ variable
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;    
  nodeHandle_.getParam("crit_dist", crit_dist_str_);
  return true;
}



//////* Collision avoidance logic*//////

///* Auxiliary Functions*///

float RbdLidar::getAzimuthFromCol(uint32_t col){
  if((col>(n_cols-1)) || (col<0))    ROS_ERROR("Horizontal index out of bounds. Must be in interval [0, n_cols]");

  return 360*((float(col))/float(n_cols-1));
}

std::vector<float> RbdLidar::getCriticalAzimuths(uint32_t* row){
  std::vector<float> crit_Az;
  
  for(uint32_t col = 0; col < n_cols; col++){
    if((row[col] < critical_distance_mm) && !(row[col]==0)){        // ignore empty points
      crit_Az.push_back(getAzimuthFromCol(col));
      nr_crit_azimuths++;
    }
  }
  return crit_Az;
}

void RbdLidar::printList(std::list<std::vector<float> >& listOfVectors){
  uint32_t row = 0;
  for(auto vect : listOfVectors){
    if(vect.size() > 0){
      printf("Critical azimuths in row nr. %d [", row);
      for (auto element : vect){
        std::cout << element << ' ';
      }
      printf("]\n");
    }
    row++;
  }
}


///* Main callback of this node*///

void RbdLidar::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  // init
  critical_distance_mm = std::stoi(crit_dist_str_.c_str());   // convert crit_dist_str_ from parameter server to uint32_t
  nr_crit_azimuths = 0;   // reset counter for critical azimuths
  std::string file_path = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/range_list.csv";  std::ofstream myFile(file_path);  myFile << "range\n";    // init file to write coordinates of last message to range_list.csv
  collision = true;     // default: collision warning

  // get range for each point and store it in 2D array
  n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;
  uint32_t ranges[n_rows][n_cols];

  for(uint32_t row = 0; row < n_rows; row++){                 // row
    for(uint32_t col = 0; col < n_cols; col++){               // column
      uint32_t arrayPos =  row*inputPointCloud2.row_step + col*inputPointCloud2.point_step;
      uint32_t arrayPosRange = arrayPos + inputPointCloud2.fields[8].offset;       // range has an offset of 8

      uint32_t range_mm = 0;
      memcpy(&range_mm, &inputPointCloud2.data[arrayPosRange], sizeof(uint32_t));
      ranges[row][col] = range_mm;

      myFile << range_mm << std::endl;                        // write range to csv
    }
  }
  myFile.close();

  // get critical Azimuths for row in lidar scan and store it in a list
  std::list<std::vector<float> > ListCriticalAzimuths;
  for(uint32_t rowToCheck = 0; rowToCheck < n_rows; rowToCheck++){
    ListCriticalAzimuths.push_back(getCriticalAzimuths(ranges[rowToCheck]));
  }

  // no danger of collision
  if(nr_crit_azimuths < threshold_crit_azimuths){
    collision = false;
  }

  if(collision != last_collision){

    collisionSrv.request.data = collision;      // true per default
    ROS_INFO_STREAM("Call: " << (int) collisionSrv.request.data);
    collisionClient.call(collisionSrv);

    if (!collisionClient.call(collisionSrv)){
      ROS_ERROR("Failed to call service collision");
    }

  }

  last_collision = collision;
  
  

  // print number of critical azimuths, list of the azimuths and their row
  if(nr_crit_azimuths > 0){
    //ROS_INFO("\n<--Total number of critical azimuths in this message: %d-->\n", nr_crit_azimuths);
    //printList(ListCriticalAzimuths);
  }
  else{
    //ROS_INFO("\n<--No critical azimuths found in this message-->\n");
  }  
}

} /* namespace */


