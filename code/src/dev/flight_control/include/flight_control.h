#ifndef __FLIGHT_CONTROL_
#define __FLIGHT_CONTROL_

#include "ros/ros.h"

#include "airsim_ros/RotorPWM.h"
#include "quadrotor_msgs/PositionCommand.h"

class FlightControl {
public:
  FlightControl(ros::NodeHandle &);

private:
  ros::NodeHandle &nh;
  ros::ServiceClient takeoff_client, land_client;
  ros::Publisher pwm_pub;
  ros::Subscriber pos_sub;
  ros::Timer pwm_pub_timer;

  airsim_ros::RotorPWM pwm;

  void pwmPubTimerCb(const ros::TimerEvent &);
  void posSubCb(const quadrotor_msgs::PositionCommandConstPtr &);
};

#endif