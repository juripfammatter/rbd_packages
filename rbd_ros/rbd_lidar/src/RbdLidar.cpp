#include "rbd_lidar/RbdLidar.hpp"

#define deg_to_rad(deg) (deg)*deg_to_rad_factor
#define rad_to_deg(rad) (rad)*rad_to_deg_factor

namespace rbd_lidar {

bool RbdLidar::readParameters()
{
  // get from .yaml file and save it into variables
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_) ||   
      !nodeHandle_.getParam("critical_dist_back", critical_distance_back_mm_load) ||
      !nodeHandle_.getParam("critical_dist_left", critical_distance_left_mm_load) ||
      !nodeHandle_.getParam("critical_dist_front", critical_distance_front_mm_load) ||
      !nodeHandle_.getParam("critical_dist_right", critical_distance_right_mm_load) ||
      !nodeHandle_.getParam("lim_rows_back", lim_rows_back_load)) return false;
  return true;
}

RbdLidar::RbdLidar(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  // read parameters from parameter server
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  // check and set distance parameters
  if((critical_distance_back_mm_load < 0) || (critical_distance_left_mm_load < 0) ||
     (critical_distance_front_mm_load < 0)|| (critical_distance_right_mm_load < 0) ||
     (lim_rows_back <0) || (lim_rows_back > 32)){
    ROS_ERROR("Invalid (negative) value for critical distance or invalid value for lim_rows_back.");
    ros::requestShutdown();
  }
  else{
    critical_distance_back_mm = critical_distance_back_mm_load;
    critical_distance_left_mm = critical_distance_left_mm_load;
    critical_distance_front_mm = critical_distance_front_mm_load;
    critical_distance_right_mm = critical_distance_right_mm_load;
    lim_rows_back = lim_rows_back_load;
  }
  
  // parameter info
  ROS_INFO("Subscribed to topic: %s", subscriberTopic_.c_str());
  ROS_INFO("Loaded critical_dist_1 (back): %.1f mm", critical_distance_back_mm);
  ROS_INFO("Loaded critical_dist_2 (left when looking from back): %.1f mm", critical_distance_left_mm);
  ROS_INFO("Loaded critical_dist_3 (front): %.1f mm", critical_distance_front_mm);
  ROS_INFO("Loaded critical_dist_4 (right when looking from back): %.1f mm", critical_distance_right_mm);
  ROS_INFO("Loaded lim_rows_back: first %d rows", lim_rows_back);
  ROS_INFO("Successfully launched node.");

  // initialize angles for collision detection
  alpha_deg = rad_to_deg(atan(critical_distance_left_mm/critical_distance_back_mm));
  beta_deg = rad_to_deg(atan(critical_distance_left_mm/critical_distance_front_mm));
  gamma_deg = rad_to_deg(atan(critical_distance_right_mm/critical_distance_front_mm));
  delta_deg = rad_to_deg(atan(critical_distance_right_mm/critical_distance_back_mm));
  if((alpha_deg<0) || (beta_deg<0) || (gamma_deg<0) || (delta_deg<0)) ROS_ERROR("Invalid section angle");

  phi1_deg = alpha_deg;
  phi2_deg = 180 - beta_deg;
  phi3_deg = 180 + gamma_deg;
  phi4_deg = 360 - delta_deg;
  ROS_INFO("angles: phi1 %0.1f; phi2 %0.1f; phi3 %0.1f; phi4 %0.1f;", phi1_deg,phi2_deg,phi3_deg,phi4_deg);

  // publishers
  pub_PC2 = nodeHandle_.advertise<sensor_msgs::PointCloud2>("output_PC2_flagged", 1); 
  pub_PC2_scan = nodeHandle_.advertise<sensor_msgs::PointCloud2>("output_PC2_scan", 1); 

  // subscriptions
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,&RbdLidar::topicCallback, this);
  subscriber_status = nodeHandle_.subscribe("status_pub_topic", 1,&RbdLidar::statusTopicCallback, this);

  // service
  collisionClient = nodeHandle_.serviceClient<std_srvs::SetBool>("/collision");

}

RbdLidar::~RbdLidar()
{
}


//////* Collision avoidance logic*//////
std::vector<float> RbdLidar::findCollisions_getCritAzimuths(uint32_t* row){
  // PointXYZI cloud for scan
  pcl::PointXYZI newPoint;
  if(row_pcl == 15){
    for(uint32_t col = 0; col < n_cols; col++){
      uint32_t index_pcl = row_pcl*n_cols + col;

      newPoint.x = pcl_cloud->points[index_pcl].x;
      newPoint.y = pcl_cloud->points[index_pcl].y;
      newPoint.z = pcl_cloud->points[index_pcl].z;
      pcl_cloud_scan->points.push_back(newPoint);
    }
  }

  // handle critical zone
  std::vector<float> crit_Az;
  for(uint32_t col = 0; col < n_cols; col++){
    // angles in space for current point
    float phi_deg = 360*((float(col))/float(n_cols-1));
    float row_normed = abs(row_pcl-15.5);
    float theta_deg = row_normed*(22.5/15.5);     // 22.5 ° is max angle
    // range and index
    float range_mm = row[col], d_peripendicular_mm = 0;
    uint32_t index_pcl = row_pcl*n_cols + col;


    // invalid azimuths
    if((phi_deg < 0) || (phi_deg > 360)){
      ROS_ERROR("Invalid azimuth");
    }
    // section 1 (back)
    else if(((phi_deg <= phi1_deg) || (phi_deg > phi4_deg)) && (row_pcl < lim_rows_back)){
      d_peripendicular_mm = range_mm*abs(cos(deg_to_rad(phi_deg)))*abs(cos(deg_to_rad(theta_deg)));
      // too close
      if((d_peripendicular_mm < critical_distance_back_mm) && (range_mm > blind_zone)){
        crit_Az.push_back(phi_deg); nr_crit_azimuths++;

        // write flag to pcl_cloud
        pcl_cloud->points[index_pcl].intensity = 1;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].intensity = 0;
        pcl_cloud->points[index_pcl].x = 0; pcl_cloud->points[index_pcl].y = 0; pcl_cloud->points[index_pcl].z = 0;
      }
    }
    // section 2 (left when looking from back)
    else if((phi_deg > phi1_deg) && (phi_deg <= phi2_deg)){
      d_peripendicular_mm = range_mm*abs(sin(deg_to_rad(phi_deg)))*abs(cos(deg_to_rad(theta_deg)));
      // too close
      if((d_peripendicular_mm < critical_distance_left_mm) && (range_mm > blind_zone)){
        crit_Az.push_back(phi_deg); nr_crit_azimuths++;
        pcl_cloud->points[index_pcl].intensity = 1;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].intensity = 0;
        pcl_cloud->points[index_pcl].x = 0; pcl_cloud->points[index_pcl].y = 0; pcl_cloud->points[index_pcl].z = 0;
      }
    }
    // section 3  (front)
    else if((phi_deg > phi2_deg) && (phi_deg <= phi3_deg)){
      d_peripendicular_mm = range_mm*abs(cos(deg_to_rad(phi_deg)))*abs(cos(deg_to_rad(theta_deg)));
      // too close
      if((d_peripendicular_mm < critical_distance_front_mm) && (range_mm > blind_zone)){
        crit_Az.push_back(phi_deg); nr_crit_azimuths++;
        pcl_cloud->points[index_pcl].intensity = 1;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].intensity = 0;
        pcl_cloud->points[index_pcl].x = 0; pcl_cloud->points[index_pcl].y = 0; pcl_cloud->points[index_pcl].z = 0;
      }
    }
    // section 4 (right when looking from back)
    else if((phi_deg > phi3_deg) && (phi_deg <= phi4_deg)){
      d_peripendicular_mm = range_mm*abs(sin(deg_to_rad(phi_deg)))*abs(cos(deg_to_rad(theta_deg)));
      // too close
      if((d_peripendicular_mm < critical_distance_right_mm) && (range_mm > blind_zone)){
        crit_Az.push_back(phi_deg); nr_crit_azimuths++;
        pcl_cloud->points[index_pcl].intensity = 1;
      }
      // far enough
      else{
        pcl_cloud->points[index_pcl].intensity = 0;
        pcl_cloud->points[index_pcl].x = 0; pcl_cloud->points[index_pcl].y = 0; pcl_cloud->points[index_pcl].z = 0;
      }
    }
    else{
      pcl_cloud->points[index_pcl].intensity = 0;
      pcl_cloud->points[index_pcl].x = 0; pcl_cloud->points[index_pcl].y = 0; pcl_cloud->points[index_pcl].z = 0;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr RbdLidar::convertToPCL(const sensor_msgs::PointCloud2& inputPointCloud2){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(inputPointCloud2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud);
  return pcl_cloud;
}

void RbdLidar::statusTopicCallback(const std_msgs::String& message){
  std::string execution_status = message.data;
  if(execution_status == "running"){
    rbd_is_running = true;
  }
  else{
    rbd_is_running = false;
  }
}


///* Main callback of this node*///
void RbdLidar::topicCallback(const sensor_msgs::PointCloud2& inputPointCloud2)
{
  // init
  nr_crit_azimuths = 0;                                       // reset counter for critical azimuths
  std::string file_path = "/home/juri/catkin_ws/src/rbd_lidar/src/csv/range_list.csv";  std::ofstream myFile(file_path);  myFile << "range\n";    // init file to write coordinates of last message to range_list.csv
  collision = true;                                           // default: collision warning
  n_rows = inputPointCloud2.height, n_cols = inputPointCloud2.width;

  // convert PointCloud2& to pcl and generate outputPointCloud2
  pcl_cloud = convertToPCL(inputPointCloud2);
  sensor_msgs::PointCloud2::Ptr outputPointCloud2(new sensor_msgs::PointCloud2);

  // PointCloud2& for SLAM scan
  pcl_cloud_scan = convertToPCL(inputPointCloud2);
  pcl_cloud_scan->clear();
  sensor_msgs::PointCloud2::Ptr scanPointCloud2(new sensor_msgs::PointCloud2);

  //! get range for each point and store it in 2D array
  uint32_t ranges[n_rows][n_cols];

  for(uint32_t row = 0; row < n_rows; row++){                 // row
    for(uint32_t col = 0; col < n_cols; col++){               // column
      // get index of range entry
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


  // get critical Azimuths for row in lidar scan and store it in a list
  std::list<std::vector<float> > ListCriticalAzimuths;
  for(uint32_t rowToCheck = 0; rowToCheck < n_rows; rowToCheck++){
    row_pcl = rowToCheck;
    ListCriticalAzimuths.push_back(findCollisions_getCritAzimuths(ranges[rowToCheck]));
  }

  // republish modified PointCloud2
  pcl::toROSMsg(*pcl_cloud, *outputPointCloud2);
  pub_PC2.publish(outputPointCloud2);

  // publish PointCloud2 for scan
  if(rbd_is_running){
    pcl::toROSMsg(*pcl_cloud_scan, *scanPointCloud2);
    pub_PC2_scan.publish(scanPointCloud2);
  }

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

