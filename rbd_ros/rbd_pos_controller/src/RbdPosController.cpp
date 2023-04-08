#include "rbd_pos_controller/RbdPosController.hpp"

// STD
#include <string>

namespace rbd_pos_controller {

RbdPosController::RbdPosController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Create subscribers
  subscriber_ = nodeHandle_.subscribe(sub_topic, 1,&RbdPosController::actualPositionCallback, this);

  // Create publishers
  cmdVelPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmd_vel_pub_topic,1);
  statusPublisher_ = nodeHandle_.advertise<std_msgs::String>(status_pub_topic,1);

  // Create service servers/clients
  emServiceServer_ = nodeHandle_.advertiseService(emergency_stop_service,&RbdPosController::emStopCallback, this);
  posServiceServer_ = nodeHandle_.advertiseService(set_position_service,&RbdPosController::setPosCallback, this);

  poseClient_ = nodeHandle_.serviceClient<qre_msgs::SetBodyPose>(body_pose_service);

  // set status to idle
  status_message.data = "em_stop";
  statusPublisher_.publish(status_message);
  ROS_INFO("Successfully launched node.");
}

RbdPosController::~RbdPosController()
{
}



bool RbdPosController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", sub_topic) || 
      !nodeHandle_.getParam("cmd_vel_publisher_topic", cmd_vel_pub_topic) ||
      !nodeHandle_.getParam("status_publisher_topic", status_pub_topic) ||
      !nodeHandle_.getParam("body_pose_service", body_pose_service)  ||
      !nodeHandle_.getParam("emergency_stop_service", emergency_stop_service)  ||
      !nodeHandle_.getParam("set_position_service", set_position_service) )
  {
    return false;
  } 
  return true;
}

float RbdPosController::saturate(double value, double lower_limit, double upper_limit)
{
  double sat_value;

  if(value > upper_limit){
    sat_value = upper_limit;
  } else if (value < lower_limit){
    sat_value = lower_limit;
  } else{
    sat_value = value;
  }

  return sat_value;
}

void RbdPosController::envelope(double &vx, double &vy){

    double vxn = pow(vx,3)/(pow(vx,2)+pow(vy,2));
    double vyn = pow(vy,3)/(pow(vx,2)+pow(vy,2));

    vx = vxn;
    vy = vyn;

}

void RbdPosController::updateGain(void)
{
  float dx = actual_position.x-goal_position.x;
  float dy = actual_position.y-goal_position.y;

  ctrl_vel_x = saturate(kp_x*dx, -0.5, 0.5);
  ctrl_vel_y = saturate(kp_y*dy, -0.5, 0.5);

  //envelope to minimize rotational speed when moving forward
  envelope(ctrl_vel_x, ctrl_vel_y);

  //qre_ros/hlm.cpp also does control
}


void RbdPosController::actualPositionCallback(const geometry_msgs::Pose& pose)
{
  
  // main Functions are implemented here
  //update actual positions
  geometry_msgs::Point z;
  z.x = 0;
  z.y = 0;
  z.z = 0;
  actual_position = z;//pose.position;
  actual_orientation = pose.orientation;
  //ROS_INFO_STREAM("actual position [XYZ xyzq]"<<actual_position.x<<";"<<actual_position.y<<";"<<actual_position.z<<";"<<actual_orientation.x<<";"<<actual_orientation.y<<";"<<actual_orientation.z<<";"<<actual_orientation.w);
  //ROS_INFO_STREAM("goal position [XYZ RPY]"<<goal_position.x<<";"<<goal_position.y<<";"<<goal_position.z<<";"<<goal_roll<<";"<<goal_pitch<<";"<<goal_yaw);
  
  //control loop
  remaining_distance = sqrt(pow(actual_position.x-goal_position.x,2)+pow(actual_position.y-goal_position.y,2)+pow(actual_position.z-goal_position.z,2));

  while((remaining_distance > 0.1) & (!emergency_stop)){

    //Publish cmd_velocity
    vel_message.linear.x = ctrl_vel_x;
    vel_message.linear.y = ctrl_vel_y;

    cmdVelPublisher_.publish(vel_message);
    //ROS_INFO_STREAM("lin:" << vel_message.linear.x << "ang:" <<vel_message.angular.z);

    status_message.data = "executing";
    statusPublisher_.publish(status_message);
	  
  }
  
  // TBD set orientation (rotate)

  // set pody Pose

  if(!emergency_stop){
    //create request
    // if((goal_roll != 0) || (goal_pitch != 0) || (goal_yaw != 0) || (goal_position.z != 0)){
    pose_srv.request.request.roll =  goal_roll*k_roll;
    pose_srv.request.request.pitch =  goal_pitch*k_pitch;
    pose_srv.request.request.yaw =  goal_yaw*k_yaw;
    //pose_srv.request.request.body_height = goal_position.z; //-zero_height;
    //ROS_INFO_STREAM("Creating request with RPY:"<<pose_srv.request.request.roll<<";"<<pose_srv.request.request.pitch<<";"<<pose_srv.request.request.yaw);

    if (!poseClient_.call(pose_srv))
    {
      ROS_ERROR("Failed to call service set_body_pose");
    } else{
      //ROS_INFO_STREAM("Service called successfully");
    }

    // }
    ros::WallDuration(1.0).sleep();
    status_message.data = "idle";
    statusPublisher_.publish(status_message);
  } else {
    status_message.data = "em_stop";
    statusPublisher_.publish(status_message);
  }
}



bool RbdPosController::emStopCallback(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response)
{
  this->emergency_stop = request.data;

	if(emergency_stop){
    stop_message = zero_twist;
		cmdVelPublisher_.publish(stop_message);
		response.message = "emergency flag set";

    status_message.data = "em_stop";
    statusPublisher_.publish(status_message);
	} else {
		response.message = "emergency flag reset";
    status_message.data = "idle";
    statusPublisher_.publish(status_message);
	}

	response.success = true;
  return 1;
}

  bool RbdPosController::setPosCallback(rbd_msgs::SetPosition::Request& request,
                                        rbd_msgs::SetPosition::Response& response)
  {
    // Extract position and RPY
    goal_position = request.goal.position;

    tf::Quaternion q(request.goal.orientation.x, request.goal.orientation.y, request.goal.orientation.z, request.goal.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(goal_roll, goal_pitch, goal_yaw);

    if(isnan(goal_roll)) goal_roll = 0;
    if(isnan(goal_pitch)) goal_pitch = 0;
    if(isnan(goal_yaw)) goal_yaw = 0;

    //generate response
    response.message = "position set to Roll="+std::to_string(goal_roll)+" Pitch="+std::to_string(goal_pitch)+" Yaw="+std::to_string(goal_yaw);
    //response.distance = remaining_distance;
    response.success = true;
    return 1;
  }                                        


} /* namespace */
