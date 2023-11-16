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
  pos_sub =
      nh.subscribe("/planning/pos_cmd", 10, &FlightControl::posSubCb, this);
  dy_cb_f = boost::bind(&FlightControl::dyCb, this, _1, _2);
  dy_server.setCallback(dy_cb_f);

  // PID
}

void FlightControl::pwmPubTimerCb(const ros::TimerEvent &e) {}

void FlightControl::posSubCb(
  const quadrotor_msgs::PositionCommandConstPtr &msg) {
  float t_now = msg->header.stamp.toSec();
}

void FlightControl::dyCb(flight_pid::flight_pidConfig &cfg, uint32_t level) {
  ROS_INFO("PID dynamic reconfigure update.");
  pid_chain.fl_pwm.setPid(cfg.fl_pwm_p, cfg.fl_pwm_i, cfg.fl_pwm_d);
  pid_chain.fr_pwm.setPid(cfg.fr_pwm_p, cfg.fr_pwm_i, cfg.fr_pwm_d);
  pid_chain.rl_pwm.setPid(cfg.rl_pwm_p, cfg.rl_pwm_i, cfg.rl_pwm_d);
  pid_chain.rr_pwm.setPid(cfg.rr_pwm_p, cfg.rr_pwm_i, cfg.rr_pwm_d);
  pid_chain.angle_pitch.setPid(cfg.angle_pitch_p, cfg.angle_pitch_i,
                               cfg.angle_pitch_d);
  pid_chain.angle_yaw.setPid(cfg.angle_yaw_p, cfg.angle_yaw_i, cfg.angle_yaw_d);
  pid_chain.angle_roll.setPid(cfg.angle_roll_p, cfg.angle_roll_i,
                              cfg.angle_roll_d);
  pid_chain.angle_vel_pitch.setPid(cfg.angle_vel_pitch_p, cfg.angle_vel_pitch_i,
                                   cfg.angle_vel_pitch_d);
  pid_chain.angle_vel_yaw.setPid(cfg.angle_vel_yaw_p, cfg.angle_vel_yaw_i,
                                 cfg.angle_vel_yaw_d);
  pid_chain.angle_vel_roll.setPid(cfg.angle_vel_roll_p, cfg.angle_vel_roll_i,
                                  cfg.angle_vel_roll_d);
  pid_chain.vel_x.setPid(cfg.vel_x_p, cfg.vel_x_i, cfg.vel_x_d);
  pid_chain.vel_y.setPid(cfg.vel_y_p, cfg.vel_y_i, cfg.vel_y_d);
  pid_chain.vel_z.setPid(cfg.vel_z_p, cfg.vel_z_i, cfg.vel_z_d);
  pid_chain.position_x.setPid(cfg.position_x_p, cfg.position_x_i,
                              cfg.position_x_d);
  pid_chain.position_y.setPid(cfg.position_y_p, cfg.position_y_i,
                              cfg.position_y_d);
  pid_chain.position_z.setPid(cfg.position_z_p, cfg.position_z_i,
                              cfg.position_z_d);
}

PidChain::PidChain() {}

void PidChain::positionUpdate(float t_now, float x_now, float y_now, float z_now) {
  vel_x.update(t_now, position_x.update(t_now, x_now));
  vel_y.update(t_now, position_y.update(t_now, y_now));
  vel_z.update(t_now, position_z.update(t_now, z_now));
  // TODO
}

void PidChain::positionSet(float x, float y, float z) {
  position_x.setExpect(x);
  position_y.setExpect(y);
  position_z.setExpect(z);
}
void PidChain::velocitySet(float x, float y, float z) {
  vel_x.setExpect(x);
  vel_y.setExpect(y);
  vel_z.setExpect(z);
}
void PidChain::angleSet(float pitch, float yaw, float roll) {
  angle_pitch.setExpect(pitch);
  angle_yaw.setExpect(yaw);
  angle_roll.setExpect(roll);
}
