#ifndef __FLIGHT_CONTROL_
#define __FLIGHT_CONTROL_

#include "ros/ros.h"
#include <cstdint>
#include <dynamic_reconfigure/server.h>

#include "airsim_ros/RotorPWM.h"
#include "pid.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <flight_control/flight_pidConfig.h>

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
  PID::Pid pid_fl_pwm, pid_fr_pwm, pid_rl_pwm,
      pid_rr_pwm; // front rear left right

  void dyCb(flight_pid::flight_pidConfig &, uint32_t);
};

#endif