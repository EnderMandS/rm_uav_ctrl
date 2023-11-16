#include "flight_control.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/RotorPWM.h"
#include "airsim_ros/Takeoff.h"
#include <boost/bind/bind.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "flight_control");
  ros::NodeHandle nh;
  FlightControl fc(nh);
  ROS_INFO("Flight control start.");
  ros::spin();
  return 0;
}

FlightControl::FlightControl(ros::NodeHandle &nh) : nh(nh) {
  // ROS
  takeoff_client =
      nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
  land_client = nh.serviceClient<airsim_ros::Land>("/airsim_node/drone_1/land");
  pwm_pub =
      nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm", 10);
  pwm_pub_timer = nh.createTimer(ros::Duration(1.f / 200),
                                 &FlightControl::pwmPubTimerCb, this);
  pos_sub = nh.subscribe("/planning/pos_cmd", 10, &FlightControl::posSubCb, this);
  dy_cb_f = boost::bind(&FlightControl::dyCb, this, _1, _2);
  dy_server.setCallback(dy_cb_f);

  // PID
  
}

void FlightControl::pwmPubTimerCb(const ros::TimerEvent &e) {
  static float t_last = ros::Time::now().toSec();
  float t_now = ros::Time::now().toSec();
  float dt = t_now-t_last;
  t_last = t_now;
}

void FlightControl::posSubCb(const quadrotor_msgs::PositionCommandConstPtr &msg) {}

void FlightControl::dyCb(flight_pid::flight_pidConfig &cfg, uint32_t level) {
  ROS_INFO("PID dynamic reconfigure update.");
}