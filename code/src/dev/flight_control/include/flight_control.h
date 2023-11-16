#ifndef __FLIGHT_CONTROL_
#define __FLIGHT_CONTROL_

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <cmath>
#include <cstdint>
#include <dynamic_reconfigure/server.h>

#include "airsim_ros/RotorPWM.h"
#include "pid.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <flight_control/flight_pidConfig.h>

class PidChain {
public:
  PidChain();
  void positionSet(float x, float y, float z);
  void velocitySet(float x, float y, float z);
  void angleSet(float pitch, float yaw, float roll);
  void positionUpdate(float t_now, float x_now, float y_now, float z_now);
  
  PID::Pid fl_pwm, fr_pwm, rl_pwm, rr_pwm; // front rear left right
  PID::Pid angle_pitch, angle_yaw, angle_roll;
  PID::Pid angle_vel_pitch, angle_vel_yaw, angle_vel_roll;
  // PID::Pid acc_x, acc_y, acc_z;
  PID::Pid vel_x, vel_y, vel_z;
  PID::Pid position_x, position_y, position_z;
};

class FlightControl {
public:
  FlightControl(ros::NodeHandle &);

private:
  // ROS
  ros::NodeHandle &nh;
  ros::ServiceClient takeoff_client, land_client;
  ros::Publisher pwm_pub;
  ros::Subscriber pos_sub;
  ros::Timer pwm_pub_timer;

  airsim_ros::RotorPWM pwm;

  void pwmPubTimerCb(const ros::TimerEvent &);
  void posSubCb(const quadrotor_msgs::PositionCommandConstPtr &);

  // PID
  dynamic_reconfigure::Server<flight_pid::flight_pidConfig> dy_server;
  dynamic_reconfigure::Server<flight_pid::flight_pidConfig>::CallbackType dy_cb_f;
  PidChain pid_chain;

  void dyCb(flight_pid::flight_pidConfig &, uint32_t);
};

#endif