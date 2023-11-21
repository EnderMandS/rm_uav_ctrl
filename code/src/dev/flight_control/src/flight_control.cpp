#include "flight_control.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/Takeoff.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include <boost/bind/bind.hpp>
#include <cmath>

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
      nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);
  cmd_pub_timer = nh.createTimer(ros::Duration(1.f/200),
                                 &FlightControl::cmdPubTimerCb, this);
  angle_rate_pub = nh.advertise<airsim_ros::AngleRateThrottle>(
      "/airsim_node/drone_1/angle_rate_throttle_frame", 10);
  pos_sub = nh.subscribe("/position_cmd", 10, &FlightControl::posSubCb, this);
  odom_sub = nh.subscribe("/odom_nav", 10, &FlightControl::odomCb, this);
  dy_cb_f = boost::bind(&FlightControl::dyCb, this, _1, _2);
  dy_server.setCallback(dy_cb_f);
}

void FlightControl::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  pid_chain.cal_lock.lock();
  pid_chain.position_x.now = msg->pose.pose.position.x;
  pid_chain.position_y.now = msg->pose.pose.position.y;
  pid_chain.position_z.now = msg->pose.pose.position.z;
  pid_chain.vel_x.now = msg->twist.twist.linear.x;
  pid_chain.vel_y.now = msg->twist.twist.linear.y;
  pid_chain.vel_z.now = msg->twist.twist.linear.z;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double r, p, y;
  tf::Matrix3x3(quat).getRPY(r, p, y);
  pid_chain.angle_roll.now = r / (2 * M_2_PI) * 360;
  pid_chain.angle_pitch.now = p / (2 * M_2_PI) * 360;
  pid_chain.angle_yaw.now = y / (2 * M_2_PI) * 360;

  pid_chain.angle_vel_pitch.now =
      msg->twist.twist.angular.y / (2 * M_2_PI) * 360;
  pid_chain.angle_vel_yaw.now = msg->twist.twist.angular.z / (2 * M_2_PI) * 360;
  pid_chain.angle_vel_roll.now =
      msg->twist.twist.angular.x / (2 * M_2_PI) * 360;
  pid_chain.cal_lock.unlock();

  pid_chain.velocityYawUpdate(msg->header.stamp.now().toSec());
}

void FlightControl::cmdPubTimerCb(const ros::TimerEvent &e) {
  pid_chain.velocityYawUpdate(e.current_real.now().toSec());

  pid_chain.cal_lock.lock();
  angle_rate.pitchRate = pid_chain.angle_vel_pitch.expect;
  angle_rate.yawRate = pid_chain.angle_vel_yaw.expect;
  angle_rate.rollRate = pid_chain.angle_vel_roll.expect;
  angle_rate.throttle = pid_chain.thrust.expect;
  pid_chain.cal_lock.unlock();
  angle_rate_pub.publish(angle_rate);

  // 2 0
  // 1 3
  // pid_chain.cal_lock.lock();
  // pwm.rotorPWM0 = pid_chain.vel_z.out - pid_chain.angle_vel_pitch.out -
  //                 pid_chain.angle_vel_roll.out - pid_chain.angle_vel_yaw.out;
  // pwm.rotorPWM1 = pid_chain.vel_z.out + pid_chain.angle_vel_pitch.out +
  //                 pid_chain.angle_vel_roll.out - pid_chain.angle_vel_yaw.out;
  // pwm.rotorPWM2 = pid_chain.vel_z.out - pid_chain.angle_vel_pitch.out +
  //                 pid_chain.angle_vel_roll.out + pid_chain.angle_vel_yaw.out;
  // pwm.rotorPWM3 = pid_chain.vel_z.out + pid_chain.angle_vel_pitch.out -
  //                 pid_chain.angle_vel_roll.out + pid_chain.angle_vel_yaw.out;
  // pid_chain.cal_lock.unlock();

  // pwm.header = std_msgs::Header();
  // pwm_pub.publish(pwm);
}

void FlightControl::posSubCb(
    const quadrotor_msgs::PositionCommandConstPtr &msg) {
  pid_chain.velocitySet(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  pid_chain.cal_lock.lock();
  pid_chain.angle_vel_yaw.expect = (msg->yaw_dot);
  if (msg->trajectory_flag==quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EXEC) {
    pid_chain.ctrl_enable = true;
  }
  else {
    pid_chain.ctrl_enable = false;
  }
  pid_chain.cal_lock.unlock();
  pid_chain.velocityYawUpdate(msg->header.stamp.toSec());
}

void FlightControl::dyCb(flight_pid::flight_pidConfig &cfg, uint32_t level) {
  ROS_INFO("PID dynamic reconfigure update.");
  pid_chain.cal_lock.lock();
  pid_chain.thrust.setPid(cfg.thrust_p, cfg.thrust_i, cfg.thrust_d);
  pid_chain.thrust.setMax(cfg.thrust_out_max, cfg.thrust_p_max,
                          cfg.thrust_i_max, cfg.thrust_d_max);

  // pid_chain.fl_pwm.setPid(cfg.fl_pwm_p, cfg.fl_pwm_i, cfg.fl_pwm_d);
  // pid_chain.fr_pwm.setPid(cfg.fr_pwm_p, cfg.fr_pwm_i, cfg.fr_pwm_d);
  // pid_chain.rl_pwm.setPid(cfg.rl_pwm_p, cfg.rl_pwm_i, cfg.rl_pwm_d);
  // pid_chain.rr_pwm.setPid(cfg.rr_pwm_p, cfg.rr_pwm_i, cfg.rr_pwm_d);

  pid_chain.position_x.setPid(cfg.position_x_p, cfg.position_x_i,
                              cfg.position_x_d);
  pid_chain.position_x.setMax(cfg.position_x_out_max, cfg.position_x_p_max,
                              cfg.position_x_i_max, cfg.position_x_d_max);
  pid_chain.position_y.setPid(cfg.position_y_p, cfg.position_y_i,
                              cfg.position_y_d);
  pid_chain.position_y.setMax(cfg.position_y_out_max, cfg.position_y_p_max,
                              cfg.position_y_i_max, cfg.position_y_d_max);
  pid_chain.position_z.setPid(cfg.position_z_p, cfg.position_z_i,
                              cfg.position_z_d);
  pid_chain.position_z.setMax(cfg.position_z_out_max, cfg.position_z_p_max,
                              cfg.position_z_i_max, cfg.position_z_d_max);

  pid_chain.vel_x.setPid(cfg.vel_x_p, cfg.vel_x_i, cfg.vel_x_d);
  pid_chain.vel_x.setMax(cfg.vel_x_out_max, cfg.vel_x_p_max, cfg.vel_x_i_max,
                         cfg.vel_x_d_max);
  pid_chain.vel_y.setPid(cfg.vel_y_p, cfg.vel_y_i, cfg.vel_y_d);
  pid_chain.vel_y.setMax(cfg.vel_y_out_max, cfg.vel_y_p_max, cfg.vel_y_i_max,
                         cfg.vel_y_d_max);
  pid_chain.vel_z.setPid(cfg.vel_z_p, cfg.vel_z_i, cfg.vel_z_d);
  pid_chain.vel_z.setMax(cfg.vel_z_out_max, cfg.vel_z_p_max, cfg.vel_z_i_max,
                         cfg.vel_z_d_max);

  pid_chain.angle_pitch.setPid(cfg.angle_pitch_p, cfg.angle_pitch_i,
                               cfg.angle_pitch_d);
  pid_chain.angle_pitch.setMax(cfg.angle_pitch_out_max, cfg.angle_pitch_p_max,
                               cfg.angle_pitch_i_max, cfg.angle_pitch_d_max);
  pid_chain.angle_yaw.setPid(cfg.angle_yaw_p, cfg.angle_yaw_i, cfg.angle_yaw_d);
  pid_chain.angle_yaw.setMax(cfg.angle_yaw_out_max, cfg.angle_yaw_p_max,
                             cfg.angle_yaw_i_max, cfg.angle_yaw_d_max);
  pid_chain.angle_roll.setPid(cfg.angle_roll_p, cfg.angle_roll_i,
                              cfg.angle_roll_d);
  pid_chain.angle_roll.setMax(cfg.angle_roll_out_max, cfg.angle_roll_p_max,
                              cfg.angle_roll_i_max, cfg.angle_roll_d_max);

  pid_chain.angle_vel_pitch.setPid(cfg.angle_vel_pitch_p, cfg.angle_vel_pitch_i,
                                   cfg.angle_vel_pitch_d);
  pid_chain.angle_vel_pitch.setMax(
      cfg.angle_vel_pitch_out_max, cfg.angle_vel_pitch_p_max,
      cfg.angle_vel_pitch_i_max, cfg.angle_vel_pitch_d_max);
  pid_chain.angle_vel_yaw.setPid(cfg.angle_vel_yaw_p, cfg.angle_vel_yaw_i,
                                 cfg.angle_vel_yaw_d);
  pid_chain.angle_vel_yaw.setMax(
      cfg.angle_vel_yaw_out_max, cfg.angle_vel_yaw_p_max,
      cfg.angle_vel_yaw_i_max, cfg.angle_vel_yaw_d_max);
  pid_chain.angle_vel_roll.setPid(cfg.angle_vel_roll_p, cfg.angle_vel_roll_i,
                                  cfg.angle_vel_roll_d);
  pid_chain.angle_vel_roll.setMax(
      cfg.angle_vel_roll_out_max, cfg.angle_vel_roll_p_max,
      cfg.angle_vel_roll_i_max, cfg.angle_vel_roll_d_max);
  pid_chain.cal_lock.unlock();
}

/* ---------------------------------
 * The following is class PidChain
 * ---------------------------------
 */

PidChain::PidChain() {}

void PidChain::positionUpdate(double t_now, double x_now, double y_now,
                              double z_now) {
  cal_lock.lock();
  vel_x.update(t_now, position_x.update(t_now, x_now));
  vel_y.update(t_now, position_y.update(t_now, y_now));
  vel_z.update(t_now, position_z.update(t_now, z_now));
  cal_lock.unlock();
  // TODO
}
void PidChain::velocityYawUpdate(double t_now) {
  // expects were set in function FlightControl::posSubCb()
  // measure were set in function FlightControl::odomCb()
  cal_lock.lock();
  angle_pitch.expect = vel_x.update(t_now);
  angle_vel_pitch.expect = angle_pitch.update(t_now);
  // angle_vel_pitch.update(t_now);

  // angle_vel_yaw.update(t_now);

  angle_roll.expect = vel_y.update(t_now);
  angle_vel_roll.expect = angle_roll.update(t_now);
  // angle_vel_roll.update(t_now);

  thrust.expect = vel_z.update(t_now);
  // thrust.update(t_now);
  cal_lock.unlock();
}

inline void PidChain::positionSet(double x, double y, double z) {
  cal_lock.lock();
  position_x.setExpect(x);
  position_y.setExpect(y);
  position_z.setExpect(z);
  cal_lock.unlock();
}
inline void PidChain::velocitySet(double x, double y, double z) {
  cal_lock.lock();
  vel_x.setExpect(x);
  vel_y.setExpect(y);
  vel_z.setExpect(z);
  cal_lock.unlock();
}
inline void PidChain::velocityYawSet(double x, double y, double z, double yaw) {
  cal_lock.lock();
  vel_x.setExpect(x);
  vel_y.setExpect(y);
  vel_z.setExpect(z);
  angle_yaw.setExpect(yaw);
  cal_lock.unlock();
}
inline void PidChain::angleSet(double pitch, double yaw, double roll) {
  cal_lock.lock();
  angle_pitch.setExpect(pitch);
  angle_yaw.setExpect(yaw);
  angle_roll.setExpect(roll);
  cal_lock.unlock();
}
