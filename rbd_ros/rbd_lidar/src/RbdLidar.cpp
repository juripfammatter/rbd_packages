#include "rbd_lidar/RbdLidar.hpp"

#define deg_to_rad(deg) (deg)*deg_to_rad_factor


namespace rbd_lidar {

RbdLidar::RbdLidar(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  // read parameters from parameter server
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  // check and set distance parameters
  if((critical_distance_sect1_sect3_mm_load < 0) || (critical_distance_sect2_sect4_mm < 0)){
    ROS_ERROR("Invalid (negative) value for critical_distance_sect1_sect3_mm_load or critical_distance_sect2_sect4_mm.");
    ros::requestShutdown();
  }
  else{
    critical_distance_sect1_sect3_mm = critical_distance_sect1_sect3_mm_load;
    critical_distance_sect2_sect4_mm = critical_distance_sect2_sect4_mm_load;
  }
  
  // handle subscription and service
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdLidar::topicCallback, this);
  collisionClient = nodeHandle_.serviceClient<std_srvs::SetBool>("collision");

  // parameter info
  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  ROS_INFO("Loaded critical_dist_1_3_str: %d mm", critical_distance_sect1_sect3_mm);
  ROS_INFO("Loaded critical_dist_2_4_str: %d mm", critical_distance_sect2_sect4_mm);
  ROS_INFO("Successfully launched node.");

  // initialize angles for collision detection
  alpha = atan2((w_dog/2), (l_dog/2))* 180/M_PI;
  beta = atan2((l_dog/2), (w_dog/2))* 180/M_PI;
  phi1 = alpha;
  phi2 = phi1 + 2*beta;
  phi3 = phi2 + 2*alpha;
  phi4 = phi3 + 2*beta;


  pub_PC2 = nodeHandle_.advertise<sensor_msgs::PointCloud2>("output_PC2_flagged", 1);


}

RbdLidar::~RbdLidar()
{
}

bool RbdLidar::readParameters()
{
  // get from .yaml file and save it into variables
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_) ||   
      !nodeHandle_.getParam("critical_dist_1_3", critical_distance_sect1_sect3_mm_load) ||
      !nodeHandle_.getParam("critical_dist_2_4", critical_distance_sect2_sect4_mm_load)) return false;
  return true;
}



//////* Collision avoidance logic*//////
///* Auxiliary Functions*///
float RbdLidar::getAzimuthDegFromCol(uint32_t col){
  if((col>(n_cols-1)) || (col<0))    ROS_ERROR("Horizontal index out of bounds. Must be in interval [0, n_cols]");

  return 360*((float(col))/float(n_cols-1));
}

std::vector<float> RbdLidar::getCriticalAzimuthsDeg(uint32_t* row, uint32_t rowindex){
  std::vector<float> crit_Az;

  for(uint32_t col = 0; col < n_cols; col++){
    float phi_deg = getAzimuthDegFromCol(col);
    float range_mm = row[col], d_peripendicular_mm;
    uint32_t index_pcl = row_pcl*n_cols + col;

    // invalid azimuths
    if((phi_deg < 0) || (phi_deg > 360)){
      ROS_ERROR("Invalid azimuth");
    }
    // Section 1 (back) and 3 (front)
    else if(((phi_deg <= phi1) || (phi_deg > phi4)) || ((phi_deg > phi2) && (phi_deg <= phi3))){
      d_peripendicular_mm = range_mm*abs(cos(deg_to_rad(phi_deg)));
      // critically close
      if((d_peripendicular_mm < critical_distance_sect1_sect3_mm) && (d_peripendicular_mm > blind_zone)){
        crit_Az.push_back(phi_deg);
        nr_crit_azimuths++;

        // write flag to pcl_cloud
        pcl_cloud->points[index_pcl].r = 255;
        pcl_cloud->points[index_pcl].g = 0;
        pcl_cloud->points[index_pcl].b = 0;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].r = 0;
        pcl_cloud->points[index_pcl].g = 255;
        pcl_cloud->points[index_pcl].b = 0;
      }
    }
    // Section 2 (left side when looking from behind) and 4 (right side " )
    else if(((phi_deg > phi1) && (phi_deg <= phi2)) || ((phi_deg > phi3) && (phi_deg < phi4))){
      d_peripendicular_mm = range_mm*abs(sin(deg_to_rad(phi_deg)));
      // critically close
      if((d_peripendicular_mm < critical_distance_sect2_sect4_mm) && (d_peripendicular_mm > blind_zone)){
        crit_Az.push_back(phi_deg);
        nr_crit_azimuths++;

        // write flag to pcl_cloud
        pcl_cloud->points[index_pcl].r = 255;
        pcl_cloud->points[index_pcl].g = 0;
        pcl_cloud->points[index_pcl].b = 0;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].r = 0;
        pcl_cloud->points[index_pcl].g = 255;
        pcl_cloud->points[index_pcl].b = 0;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RbdLidar::convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(inputPointCloud2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud);
  return pcl_cloud;
}

///* Main callback of this node*///
void RbdLidar::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  // init
  nr_crit_azimuths = 0;                 // reset counter for critical azimuths
  std::string file_path = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/range_list.csv";  std::ofstream myFile(file_path);  myFile << "range\n";    // init file to write coordinates of last message to range_list.csv
  collision = true;                     // default: collision warning

  // convert PointCloud2& to pcl
  pcl_cloud = convertToPCL(inputPointCloud2);

  //! get range for each point and store it in 2D array
  n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;
  uint32_t ranges[n_rows][n_cols];

  for(uint32_t row = 0; row < n_rows; row++){                 // row
    //float x_pcl,y_pcl,z_pcl;
    for(uint32_t col = 0; col < n_cols; col++){               // column
      // get index of range
      uint32_t arrayPos =  row*inputPointCloud2.row_step + col*inputPointCloud2.point_step;
      uint32_t arrayPosRange = arrayPos + inputPointCloud2.fields[8].offset;       // range is at position 8

      // store range in 2D array
      uint32_t range_mm = 0;
      memcpy(&range_mm, &inputPointCloud2.data[arrayPosRange], sizeof(uint32_t));
      ranges[row][col] = range_mm;

      // write range to csv
      myFile << range_mm << std::endl;                        
    }
  }
  myFile.close();

  // modified PointCloud2
  sensor_msgs::PointCloud2::Ptr outputPointCloud2(new sensor_msgs::PointCloud2);

  // get critical Azimuths for row in lidar scan and store it in a list
  std::list<std::vector<float> > ListCriticalAzimuths;
  for(uint32_t rowToCheck = 0; rowToCheck < n_rows; rowToCheck++){
    row_pcl = rowToCheck;
    ListCriticalAzimuths.push_back(getCriticalAzimuthsDeg(ranges[rowToCheck], rowToCheck));
  }

  // republish modified PointCloud2
  pcl::toROSMsg(*pcl_cloud, *outputPointCloud2);
  pub_PC2.publish(outputPointCloud2);

  // collision?
  if(nr_crit_azimuths < threshold_crit_azimuths){
    collision = false;                          // no danger of collision
  }

  if(collision != last_collision){              // call service if state has changed
    collisionSrv.request.data = collision;      // true per default
    ROS_INFO_STREAM("Call: " << (int) collisionSrv.request.data);
    //collisionClient.call(collisionSrv);

    if (!collisionClient.call(collisionSrv)){
      //ROS_ERROR("Failed to call service collision");
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


