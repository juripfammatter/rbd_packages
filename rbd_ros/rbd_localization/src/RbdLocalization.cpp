#include "rbd_localization/RbdLocalization.hpp"

// STD
#include <string>

namespace rbd_localization {

RbdLocalization::RbdLocalization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  statesubscriber_ = nodeHandle_.subscribe(stateSubscriberTopic_, 1,&RbdLocalization::stateCallback, this);
  imuZedSubscriber_ = nodeHandle_.subscribe(imuZedSubscriberTopic_, 1,&RbdLocalization::imuZedCallback, this);

  positionPublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>(positionPublisherTopic_,1);
  imuQrePublisher_ = nodeHandle_.advertise<sensor_msgs::Imu>(imuQrePublisherTopic_,1);
  imuZedPublisher_ = nodeHandle_.advertise<sensor_msgs::Imu>(imuZedPublisherTopic_,1);
  // serviceServer_ = nodeHandle_.advertiseService("get_average",&RbdLocalization::serviceCallback, this);

  ROS_INFO("Successfully launched node.");

  t_old_qre = ros::WallTime::now();
  t_old_zed = ros::WallTime::now();
}

RbdLocalization::~RbdLocalization()
{
}



bool RbdLocalization::readParameters()
{
  if ((!nodeHandle_.getParam("state_subscriber_topic", stateSubscriberTopic_)) ||
      (!nodeHandle_.getParam("imu_zed_subscriber_topic", imuZedSubscriberTopic_))||
      (!nodeHandle_.getParam("position_publisher_topic", positionPublisherTopic_))||
      (!nodeHandle_.getParam("imu_qre_publisher_topic", imuQrePublisherTopic_))||
      (!nodeHandle_.getParam("imu_zed_publisher_topic", imuZedPublisherTopic_))){
    return false;
  } else {
    return true;
  }
}

// Pass by reference!!
void RbdLocalization::toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose)
  {
      msg_pose.position.x = tf_pose.getOrigin().getX();
      msg_pose.position.y = tf_pose.getOrigin().getY();
      msg_pose.position.z = tf_pose.getOrigin().getZ();

      tf::Quaternion orientation = tf_pose.getRotation();
      msg_pose.orientation.w = orientation.getW();
      msg_pose.orientation.x = orientation.getX();
      msg_pose.orientation.y = orientation.getY();
      msg_pose.orientation.z = orientation.getZ();
  }

void RbdLocalization::updateState(const unitree_legged_msgs::HighState& state){
  for(int i=0;i<3;i++){
    imu_acc[i] = state.imu.accelerometer.at(i);
    imu_gyr[i] = state.imu.gyroscope.at(i);
    imu_quat[i] = state.imu.quaternion.at(i);
  }
  imu_quat[3] = state.imu.quaternion.at(3);
  imu_temp = state.imu.temperature;

}

float RbdLocalization::lowPassFilter(const float u, float& u_old, float Ts){
  float tau = 0.3;
  float K = 1.0;
  float b1 = K*Ts/(tau+Ts);
  float a0 = -tau/(tau+Ts);

  float filter_y_new = b1*u-a0*u_old;
  u_old = filter_y_new;
  return filter_y_new;

}

void RbdLocalization::imuIntegrate(const unitree_legged_msgs::HighState& state, tf::Pose& pose){
    //dt
    ros::WallTime t_new = ros::WallTime::now();
    float dt = (t_new - t_old_qre).toSec();
    t_old_qre = t_new;

    //integrate
    if(dt <= 1){
      //filter imu values
      float imu_x = lowPassFilter(state.imu.accelerometer.at(0)+0.01, imu_qre_filter_x_old, dt);
      float imu_y = lowPassFilter(state.imu.accelerometer.at(1)-0.11, imu_qre_filter_y_old, dt);
      float imu_z = lowPassFilter(state.imu.accelerometer.at(2)-9.81-0.145, imu_qre_filter_z_old, dt);

      // publish filtered values
      imu_qre_message.linear_acceleration.x = imu_x;
      imu_qre_message.linear_acceleration.y = imu_y;
      imu_qre_message.linear_acceleration.z = imu_z;
      imuQrePublisher_.publish(imu_qre_message);

      //integration
      float x_new = x_old + 0.5 * state.imu.accelerometer.at(0) * pow(dt,2);
      float y_new = y_old + 0.5 * state.imu.accelerometer.at(1) * pow(dt,2);
      float z_new = z_old + 0.5 * (state.imu.accelerometer.at(2)-9.81-0.145) * pow(dt,2);

      x_old = x_new;
      y_old = y_new;
      z_old = z_new;

      //set origin.x...
      pose.setOrigin(tf::Vector3(x_new, y_new, z_new));
    }
}

void RbdLocalization::stateCallback(const unitree_legged_msgs::HighState& state)
{
  // read and integrate imu position
  tf::Pose tf_actual_position;
  imuIntegrate(state, tf_actual_position);

  // read imu orientation
  updateState(state);
  tf::Quaternion q(imu_quat[1],imu_quat[2],imu_quat[3],imu_quat[0]);      //tf:[x,y,z,w], unitree:[w,x,y,z]

  //q.setRPY(0.0,0.0,0.0);
  //tf_actual_position.setOrigin(tf::Vector3(0,0,0));
  tf_actual_position.setRotation(q);
  toMsgPose(tf_actual_position, position_message);

  positionPublisher_.publish(position_message);
}

void RbdLocalization::imuZedCallback(const sensor_msgs::Imu& imu){
    //dt
    ros::WallTime t_new = ros::WallTime::now();
    float dt = (t_new - t_old_zed).toSec();
    t_old_zed = t_new;

    float imu_x = lowPassFilter(imu.linear_acceleration.x, imu_zed_filter_x_old, dt);
    float imu_y = lowPassFilter(imu.linear_acceleration.y, imu_zed_filter_y_old, dt);
    float imu_z = lowPassFilter(imu.linear_acceleration.z-9.81, imu_zed_filter_z_old, dt);

    // publish filtered values
    imu_zed_message.linear_acceleration.x = imu_x;
    imu_zed_message.linear_acceleration.y = imu_y;
    imu_zed_message.linear_acceleration.z = imu_z;
    imuZedPublisher_.publish(imu_zed_message);
}

// bool RbdLocalization::serviceCallback(std_srvs::Trigger::Request& request,
//                                          std_srvs::Trigger::Response& response)
// {
//   response.success = true;
//   response.message = "message";
//   return true;
// }

} /* namespace */
