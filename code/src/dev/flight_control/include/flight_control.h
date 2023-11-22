#ifndef __FLIGHT_CONTROL_
#define __FLIGHT_CONTROL_

#include "airsim_ros/AngleRateThrottle.h"
#include "airsim_ros/RotorPWM.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "pid.h"
#include "quadrotor_msgs/FsmCommand.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "ros/ros.h"
#include <cmath>
#include <cstdint>
#include <dynamic_reconfigure/server.h>
#include <flight_control/flight_pidConfig.h>
#include <mutex>
#include "sensor_msgs/Imu.h"

class PidChain {
public:
  PidChain(ros::NodeHandle &);
  inline void positionSet(double x, double y, double z);
  inline void velocitySet(double x, double y, double z);
  inline void accelSet(double x, double y, double z);
  inline void velocityYawSet(double x, double y, double z, double yaw);
  inline void angleSet(double pitch, double yaw, double roll);
  void positionUpdate(double x_now, double y_now, double z_now);
  void velocityYawUpdate();
  void accelYawUpdate();
  inline void reset();
  void pubPidDebug();

  PID::Pid angle_pitch, angle_yaw, angle_roll;
  PID::Pid angle_vel_pitch, angle_vel_yaw, angle_vel_roll;
  PID::Pid thrust;
  PID::Pid acc_x, acc_y, acc_z;
  PID::Pid vel_x, vel_y, vel_z;
  PID::Pid position_x, position_y, position_z;
  std::mutex cal_lock;
  bool ctrl_enable = false;

private:
  ros::Publisher debug_info_pub;
};

class FlightControl {
public:
  FlightControl(ros::NodeHandle &);

private:
  // ROS
  ros::NodeHandle &nh;
  ros::ServiceClient takeoff_client, land_client;
  ros::Publisher pwm_pub, angle_rate_pub;
  ros::Subscriber pos_sub, odom_sub, fsm_cmd_sub;
  ros::Timer cmd_pub_timer;
  nav_msgs::Odometry odom;

  airsim_ros::RotorPWM pwm;
  airsim_ros::AngleRateThrottle angle_rate;

  void cmdPubTimerCb(const ros::TimerEvent &);
  void odomCb(const nav_msgs::OdometryConstPtr &);
  void imuCb(const sensor_msgs::ImuConstPtr &);
  void posSubCb(const quadrotor_msgs::PositionCommandConstPtr &);
  void fsmCmdCb(const quadrotor_msgs::FsmCommandConstPtr &);

  // PID
  dynamic_reconfigure::Server<flight_pid::flight_pidConfig> dy_server;
  dynamic_reconfigure::Server<flight_pid::flight_pidConfig>::CallbackType
      dy_cb_f;
  PidChain pid_chain;

  void dyCb(flight_pid::flight_pidConfig &, uint32_t);
};

#endif