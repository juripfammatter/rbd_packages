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
      !nodeHandle_.getParam("set_position_service", set_position_service)  ||
      !nodeHandle_.getParam("kp_angular", kp_angular) )
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

// void RbdPosController::updateGain(void)
// {
//   float dx = actual_position.x-goal_position.x;
//   float dy = actual_position.y-goal_position.y;

//   ctrl_vel_x = saturate(kp_x*dx, -0.5, 0.5);
//   ctrl_vel_y = saturate(kp_y*dy, -0.5, 0.5);

//   //envelope to minimize rotational speed when moving forward
//   envelope(ctrl_vel_x, ctrl_vel_y);

//   //qre_ros/hlm.cpp also does control
// }

void RbdPosController::quat2eul(const geometry_msgs::Pose& pose, std::vector<double> &rpy){

  tf::Quaternion tf_orientation;
  tf_orientation.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 tf_m(tf_orientation);

  tf_m.getRPY(rpy[0], rpy[1], rpy[2]);
  if(isnan(rpy[0])) rpy[0] = 0;
  if(isnan(rpy[1])) rpy[1] = 0;
  if(isnan(rpy[2])) rpy[2] = 0;


}


void RbdPosController::actualPositionCallback(const geometry_msgs::Pose& pose)
{
  /* Get yaw angle of actual pose */
  std::vector<double> pose_rpy = std::vector<double> (3);
  quat2eul(pose,pose_rpy);

  /* Calculate position error */
  float e_x = goal_position.x-pose.position.x;
  float e_y = goal_position.y- pose.position.y;
  float rho = sqrt(e_x*e_x+e_y*e_y);

  /* Calculate helper angles (in radian) */
  float alpha = atan2(e_y, e_x);                      // angle of trajectory
  float gamma = alpha-pose_rpy[2];                    // angle for pre rotation and during linear movement
  float delta = goal_yaw-pose_rpy[2];                 // angle for final rotation

  /* Reset Velocity */
  vel_message = zero_twist;

  
  /*************************** State machine ****************************/
  /* 1. Wait for pose request
  *  2. Rotate to angle of trajectory 
  *  3. Move to target
  *  4. Rotate to goal (global yaw) angle
  *  5. Set goal body pose
  */
  if(emergency_stop){
    // Emergency behaviour (vel reset done by emStopCallback)
        status_message.data = "em_stop";
  } else {
    
  // switch(emergency_stop){
  //   case false: 
  //       // Control loop
        
        switch(pos_control_state){
          case WAIT_FOR_POSE:
            status_message.data = "idle";
            if(pose_requested){
              status_message.data = "running";
              // pos_control_state = PRE_ROTATION;
              pos_control_state = BODY_POSE;
              pose_requested = false;
            }
            break;


          case PRE_ROTATION:
            if(gamma >= 0.01){
              vel_message.angular.z = saturate(gamma*kp_angular, -0.5, 0.5);
            } else {
              //pos_control_state = LIN_MOVEMENT;
              pos_control_state = BODY_POSE;   // for testing (skips lin movement and post rotation)
            }
            break;


          case LIN_MOVEMENT:
            if(rho >= 0.01){
              // control loop
              vel_message.linear.x = saturate(rho*kp_linear, -0.5, 0.5);
              vel_message.linear.y = saturate(gamma*kp_angular_lin, -0.5, 0.5); //will be translated onto rotateSpeed by qre...
            } else {
              pos_control_state = POST_ROTATION;
            }
            break;


          case POST_ROTATION:
            if(delta >= 0.01){
              vel_message.angular.z = saturate(delta*kp_angular, -0.5, 0.5);
            } else {
              pos_control_state = BODY_POSE;
            }

            break;


          case BODY_POSE:
            //create request (normed to [-1,1])
            pose_srv.request.request.roll =  goal_roll*k_roll;
            pose_srv.request.request.pitch =  goal_pitch*k_pitch;
            //pose_srv.request.request.yaw =  goal_yaw*k_yaw;
            pose_srv.request.request.body_height = goal_position.z;
            //ROS_INFO_STREAM("Creating request with RPY:"<<pose_srv.request.request.roll<<";"<<pose_srv.request.request.pitch<<";"<<pose_srv.request.request.yaw);

            if (!poseClient_.call(pose_srv))
            {
              ROS_ERROR("Failed to call service set_body_pose");
            } else{
              //ROS_INFO_STREAM("Service called successfully");
            }

            ros::WallDuration(1.0).sleep();
            // set status to idle
            status_message.data = "idle";
            pos_control_state = WAIT_FOR_POSE;
            break;

        }
        cmdVelPublisher_.publish(vel_message);

  }
      // break;

  //   case true:
  //       // Emergency behaviour (vel reset done by emStopCallback)
  //       status_message.data = "em_stop";
  //       break;
  // }

  statusPublisher_.publish(status_message);
  //ROS_INFO_STREAM(emergency_stop <<pos_control_state << pose_requested);
}



bool RbdPosController::emStopCallback(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response)
{
  this->emergency_stop = request.data;

	if(emergency_stop){
    stop_message = zero_twist;
		cmdVelPublisher_.publish(stop_message);
		response.message = "emergency flag set";

    // status_message.data = "em_stop";
    // statusPublisher_.publish(status_message);
	} else {
		response.message = "emergency flag reset";
    // status_message.data = "idle";
    // statusPublisher_.publish(status_message);
	}

	response.success = true;
  return 1;
}

  bool RbdPosController::setPosCallback(rbd_msgs::SetPosition::Request& request,
                                        rbd_msgs::SetPosition::Response& response)
  {
    /* Extract position and RPY */
    goal_position = request.goal.position;

    tf::Quaternion q(request.goal.orientation.x, request.goal.orientation.y, request.goal.orientation.z, request.goal.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(goal_roll, goal_pitch, goal_yaw);

    if(isnan(goal_roll)) goal_roll = 0;
    if(isnan(goal_pitch)) goal_pitch = 0;
    if(isnan(goal_yaw)) goal_yaw = 0;

    /* generate response */
    response.message = "position set to Roll="+std::to_string(goal_roll)+" Pitch="+std::to_string(goal_pitch)+" Yaw="+std::to_string(goal_yaw);
    response.success = true;

    /* flag for state machine to start control loop*/
    pose_requested = true;
    return 1;
  }                                        


} /* namespace */
