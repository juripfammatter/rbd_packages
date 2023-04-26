#include "rbd_lidar/RbdLidar.hpp"

#define deg_to_rad(deg) ((deg) * M_PI / 180.0)


namespace rbd_lidar {

RbdLidar::RbdLidar(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdLidar::topicCallback, this);
  collisionClient = nodeHandle_.serviceClient<std_srvs::SetBool>("collision");

  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  ROS_INFO("Loaded critical_dist_1_3_str: %s mm", critical_dist_1_3_str.c_str());
  ROS_INFO("Loaded critical_dist_2_4_str: %s mm", critical_dist_2_4_str.c_str());
  ROS_INFO("Successfully launched node.");

  // initialize angles for collision detection
  alpha = atan2((w_dog/2), (l_dog/2))* 180/M_PI;
  beta = atan2((l_dog/2), (w_dog/2))* 180/M_PI;
  phi1 = alpha;
  phi2 = phi1 + 2*beta;
  phi3 = phi2 + 2*alpha;
  phi4 = phi3 + 2*beta;
  
  /* print angles
  ROS_INFO("alpha: %.3f deg", alpha);
  ROS_INFO("beta: %.3f deg", beta);
  ROS_INFO("phi1: %.3f deg", phi1);
  ROS_INFO("phi2: %.3f deg", phi2);
  ROS_INFO("phi3: %.3f deg", phi3);
  ROS_INFO("phi4: %.3f deg", phi4);*/


}

RbdLidar::~RbdLidar()
{
}

bool RbdLidar::readParameters()
{
  // get "subscriber_topic: /ouster/points" from .yaml file and save it into subscriberTopic_ variable
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;   
   
  if (!nodeHandle_.getParam("crit_dist", crit_dist_str_)) return false;
  if (!nodeHandle_.getParam("critical_dist_1_3", critical_dist_1_3_str)) return false;
  if (!nodeHandle_.getParam("critical_dist_2_4", critical_dist_2_4_str)) return false;
  return true;
}



//////* Collision avoidance logic*//////

///* Auxiliary Functions*///

float RbdLidar::getAzimuthDegFromCol(uint32_t col){
  if((col>(n_cols-1)) || (col<0))    ROS_ERROR("Horizontal index out of bounds. Must be in interval [0, n_cols]");

  return 360*((float(col))/float(n_cols-1));
}

std::vector<float> RbdLidar::getCriticalAzimuthsDeg(uint32_t* row){
  std::vector<float> crit_Az;

  // iterate through row
  for(uint32_t col = 0; col < n_cols; col++){
    float phi_deg = getAzimuthDegFromCol(col);
    float range_mm = row[col], d_perripendicular_mm;

    // invalid azimuths
    if((phi_deg < 0) || (phi_deg > 360)){
      ROS_ERROR("Invalid azimuth");
    }
    // Section 1 and 3
    else if(((phi_deg <= phi1) || (phi_deg > phi4)) || ((phi_deg > phi2) && (phi_deg <= phi3))){
      d_perripendicular_mm = range_mm*abs(cos(deg_to_rad(phi_deg)));
      if((d_perripendicular_mm < critical_distance_sect1_sect3_mm) && (d_perripendicular_mm > blind_zone)){
        crit_Az.push_back(phi_deg);
        nr_crit_azimuths++;
      }
    }
    // Section 2 and 4
    else if(((phi_deg > phi1) && (phi_deg <= phi2)) || ((phi_deg > phi3) && (phi_deg < phi4))){
      d_perripendicular_mm = range_mm*abs(sin(deg_to_rad(phi_deg)));
      if((d_perripendicular_mm < critical_distance_sect2_sect4_mm) && (d_perripendicular_mm > blind_zone)){
        crit_Az.push_back(phi_deg);
        nr_crit_azimuths++;
      }
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
  critical_distance_mm = std::stoi(crit_dist_str_.c_str());                       // convert crit_dist_str_ from parameter server to uint32_t
  critical_distance_sect1_sect3_mm = std::stoi(critical_dist_1_3_str.c_str());
  critical_distance_sect2_sect4_mm = std::stoi(critical_dist_2_4_str.c_str());
  nr_crit_azimuths = 0;                 // reset counter for critical azimuths
  std::string file_path = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/range_list.csv";  std::ofstream myFile(file_path);  myFile << "range\n";    // init file to write coordinates of last message to range_list.csv
  collisionSrv.request.data = true;     // default: collision warning

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
    ListCriticalAzimuths.push_back(getCriticalAzimuthsDeg(ranges[rowToCheck]));
  }

  // no danger of collision
  if(nr_crit_azimuths < threshold_crit_azimuths){
    collisionSrv.request.data = false;      // true per default
  }
  //collisionClient.call(collisionSrv);
  ROS_INFO_STREAM("Call: " << (int) collisionSrv.request.data);

  // print number of critical azimuths, list of the azimuths and their row
  if(nr_crit_azimuths > 0){
    ROS_INFO("\n<--Total number of critical azimuths in this message: %d-->\n", nr_crit_azimuths);
    printList(ListCriticalAzimuths);
  }
  else{
    ROS_INFO("\n<--No critical azimuths found in this message-->\n");
  }  
}

} /* namespace */


