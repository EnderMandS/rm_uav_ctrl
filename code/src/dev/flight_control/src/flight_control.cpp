#include "flight_control.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/RotorPWM.h"
#include "airsim_ros/Takeoff.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "flight_control");
  ros::NodeHandle nh;
  FlightControl fc(nh);
  ROS_INFO("Flight control start.");
  ros::spin();
  return 0;
}

FlightControl::FlightControl(ros::NodeHandle &nh) : nh(nh) {
  takeoff_client =
      nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
  land_client = nh.serviceClient<airsim_ros::Land>("/airsim_node/drone_1/land");
  pwm_pub =
      nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm", 10);
  pwm_pub_timer = nh.createTimer(ros::Duration(1.f / 200),
                                 &FlightControl::pwmPubTimerCb, this);
  pos_sub = nh.subscribe("/planning/pos_cmd", 10, &FlightControl::posSubCb, this);
}

void FlightControl::pwmPubTimerCb(const ros::TimerEvent &e) {}

void FlightControl::posSubCb(const quadrotor_msgs::PositionCommandConstPtr &msg) {}