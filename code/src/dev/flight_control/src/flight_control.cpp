#include "flight_control.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/Takeoff.h"
#include "flight_control/PidDebug.h"
#include "quadrotor_msgs/FsmCommand.h"
#include "ros/time.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <boost/bind/bind.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "flight_control");
  ros::NodeHandle nh;
  FlightControl fc(nh);
  ROS_INFO("Flight control start.");
  ros::spin();
  return 0;
}

FlightControl::FlightControl(ros::NodeHandle &nh) : nh(nh), pid_chain(nh) {
  // ROS
  takeoff_client =
      nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
  land_client = nh.serviceClient<airsim_ros::Land>("/airsim_node/drone_1/land");
  pwm_pub = nh.advertise<airsim_ros::RotorPWM>(
      "/airsim_node/drone_1/rotor_pwm_cmd", 1);
  cmd_pub_timer =
      nh.createTimer(ros::Duration(0.005), &FlightControl::cmdPubTimerCb, this);
  angle_rate_pub = nh.advertise<airsim_ros::AngleRateThrottle>(
      "/airsim_node/drone_1/angle_rate_throttle_frame", 1);
  pos_sub = nh.subscribe("/position_cmd", 1, &FlightControl::posSubCb, this);
  odom_sub = nh.subscribe("/odom_nav", 1, &FlightControl::odomCb, this);
  fsm_cmd_sub =
      nh.subscribe("/planning/fsm_cmd", 1, &FlightControl::fsmCmdCb, this);
  imu_sub = nh.subscribe("airsim_node/drone_1/imu/imu", 1,
                         &FlightControl::imuCb, this);
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
  pid_chain.angle_roll.now = r / M_PI * 180;
  pid_chain.angle_pitch.now = p / M_PI * 180;
  pid_chain.angle_yaw.now = y / M_PI * 180;

  pid_chain.angle_vel_pitch.now = msg->twist.twist.angular.y / M_PI * 180;
  pid_chain.angle_vel_yaw.now = msg->twist.twist.angular.z / M_PI * 180;
  pid_chain.angle_vel_roll.now = msg->twist.twist.angular.x / M_PI * 180;
  pid_chain.cal_lock.unlock();

  pid_chain.accelYawUpdate();
}
void FlightControl::imuCb(const sensor_msgs::ImuConstPtr &msg) {
  pid_chain.cal_lock.lock();
  pid_chain.acc_x.now = msg->linear_acceleration.x;
  pid_chain.acc_y.now = msg->linear_acceleration.y;
  pid_chain.acc_z.now = msg->linear_acceleration.z + 9.80615;
  pid_chain.cal_lock.unlock();
  pid_chain.accelYawUpdate();
}
void FlightControl::cmdPubTimerCb(const ros::TimerEvent &e) {
  // if (pid_chain.ctrl_enable == false) {
  //   return;
  // }
  pid_chain.accelYawUpdate();
  pid_chain.pubPidDebug();

  pid_chain.cal_lock.lock();
  angle_rate.pitchRate = pid_chain.angle_vel_pitch.expect / 180 * M_PI;
  angle_rate.yawRate = pid_chain.angle_vel_yaw.expect / 180 * M_PI;
  angle_rate.rollRate = pid_chain.angle_vel_roll.expect / 180 * M_PI;
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
  pid_chain.positionSet(msg->position.x, msg->position.y, msg->position.z);
  pid_chain.velocitySet(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  pid_chain.accelSet(msg->acceleration.x, msg->acceleration.y,
                     msg->acceleration.z);
  pid_chain.cal_lock.lock();
  pid_chain.angle_yaw.setExpect(msg->yaw);
  pid_chain.angle_vel_yaw.setExpect(msg->yaw_dot);
  pid_chain.cal_lock.unlock();
  pid_chain.accelYawUpdate();
}
void FlightControl::fsmCmdCb(const quadrotor_msgs::FsmCommandConstPtr &msg) {
  if (msg->trajectory_flag ==
      quadrotor_msgs::FsmCommand::TRAJECTORY_STATUS_EXEC) {
    pid_chain.ctrl_enable = true;
  } else {
    pid_chain.ctrl_enable = false;
  }
}
void FlightControl::dyCb(flight_pid::flight_pidConfig &cfg, uint32_t level) {
  ROS_INFO("PID dynamic reconfigure update.");
  pid_chain.cal_lock.lock();
  // pid_chain.thrust.setPid(cfg.thrust_p, cfg.thrust_i, cfg.thrust_d);
  // pid_chain.thrust.setMax(cfg.thrust_out_max, cfg.thrust_p_max,
  //                         cfg.thrust_i_max, cfg.thrust_d_max);

  // pid_chain.position_x.setPid(cfg.position_x_p, cfg.position_x_i,
  //                             cfg.position_x_d);
  // pid_chain.position_y.setPid(cfg.position_y_p, cfg.position_y_i,
  //                             cfg.position_y_d);
  // pid_chain.position_z.setPid(cfg.position_z_p, cfg.position_z_i,
  //                             cfg.position_z_d);
  // pid_chain.position_x.setMax(cfg.position_x_out_max, cfg.position_x_p_max,
  //                             cfg.position_x_i_max, cfg.position_x_d_max);
  // pid_chain.position_y.setMax(cfg.position_y_out_max, cfg.position_y_p_max,
  //                             cfg.position_y_i_max, cfg.position_y_d_max);
  // pid_chain.position_z.setMax(cfg.position_z_out_max, cfg.position_z_p_max,
  //                             cfg.position_z_i_max, cfg.position_z_d_max);

  // pid_chain.vel_x.setPid(cfg.vel_x_p, cfg.vel_x_i, cfg.vel_x_d);
  // pid_chain.vel_y.setPid(cfg.vel_y_p, cfg.vel_y_i, cfg.vel_y_d);
  pid_chain.vel_z.setPid(cfg.vel_z_p, cfg.vel_z_i, cfg.vel_z_d);
  // pid_chain.vel_x.setMax(cfg.vel_x_out_max, cfg.vel_x_p_max, cfg.vel_x_i_max,
  //                        cfg.vel_x_d_max);
  // pid_chain.vel_y.setMax(cfg.vel_y_out_max, cfg.vel_y_p_max, cfg.vel_y_i_max,
  //                        cfg.vel_y_d_max);
  pid_chain.vel_z.setMax(cfg.vel_z_out_max, cfg.vel_z_p_max, cfg.vel_z_i_max,
                         cfg.vel_z_d_max);

  pid_chain.acc_x.setPid(cfg.acc_x_p, cfg.acc_x_i, cfg.acc_x_d);
  pid_chain.acc_y.setPid(cfg.acc_y_p, cfg.acc_y_i, cfg.acc_y_d);
  pid_chain.acc_z.setPid(cfg.acc_z_p, cfg.acc_z_i, cfg.acc_z_d);
  pid_chain.acc_x.setMax(cfg.acc_x_out_max, cfg.acc_x_p_max, cfg.acc_x_i_max,
                         cfg.acc_x_d_max);
  pid_chain.acc_y.setMax(cfg.acc_y_out_max, cfg.acc_y_p_max, cfg.acc_y_i_max,
                         cfg.acc_y_d_max);
  pid_chain.acc_z.setMax(cfg.acc_z_out_max, cfg.acc_z_p_max, cfg.acc_z_i_max,
                         cfg.acc_z_d_max);

  pid_chain.angle_pitch.setPid(cfg.angle_pitch_p, cfg.angle_pitch_i,
                               cfg.angle_pitch_d);
  // pid_chain.angle_yaw.setPid(cfg.angle_yaw_p, cfg.angle_yaw_i,
  // cfg.angle_yaw_d);
  pid_chain.angle_roll.setPid(cfg.angle_roll_p, cfg.angle_roll_i,
                              cfg.angle_roll_d);
  pid_chain.angle_pitch.setMax(cfg.angle_pitch_out_max, cfg.angle_pitch_p_max,
                               cfg.angle_pitch_i_max, cfg.angle_pitch_d_max);
  // pid_chain.angle_yaw.setMax(cfg.angle_yaw_out_max, cfg.angle_yaw_p_max,
  //                            cfg.angle_yaw_i_max, cfg.angle_yaw_d_max);
  pid_chain.angle_roll.setMax(cfg.angle_roll_out_max, cfg.angle_roll_p_max,
                              cfg.angle_roll_i_max, cfg.angle_roll_d_max);

  // pid_chain.angle_vel_pitch.setPid(cfg.angle_vel_pitch_p,
  // cfg.angle_vel_pitch_i,
  //                                  cfg.angle_vel_pitch_d);
  // pid_chain.angle_vel_pitch.setMax(
  //     cfg.angle_vel_pitch_out_max, cfg.angle_vel_pitch_p_max,
  //     cfg.angle_vel_pitch_i_max, cfg.angle_vel_pitch_d_max);
  // pid_chain.angle_vel_yaw.setPid(cfg.angle_vel_yaw_p, cfg.angle_vel_yaw_i,
  //                                cfg.angle_vel_yaw_d);
  // pid_chain.angle_vel_yaw.setMax(
  //     cfg.angle_vel_yaw_out_max, cfg.angle_vel_yaw_p_max,
  //     cfg.angle_vel_yaw_i_max, cfg.angle_vel_yaw_d_max);
  // pid_chain.angle_vel_roll.setPid(cfg.angle_vel_roll_p, cfg.angle_vel_roll_i,
  //                                 cfg.angle_vel_roll_d);
  // pid_chain.angle_vel_roll.setMax(
  //     cfg.angle_vel_roll_out_max, cfg.angle_vel_roll_p_max,
  //     cfg.angle_vel_roll_i_max, cfg.angle_vel_roll_d_max);
  pid_chain.cal_lock.unlock();
  pid_chain.accelYawUpdate();
}

/* ---------------------------------
 * The following is class PidChain
 * ---------------------------------
 */
PidChain::PidChain(ros::NodeHandle &nh) {
  debug_info_pub = nh.advertise<flight_control::PidDebug>("/pid_debug", 10);
  reset();
  vel_z.setExpect(0.5);
}
void PidChain::positionUpdate(double x_now, double y_now, double z_now) {
  cal_lock.lock();
  // vel_x.update(t_now, position_x.update(t_now, x_now));
  // vel_y.update(t_now, position_y.update(t_now, y_now));
  // vel_z.update(t_now, position_z.update(t_now, z_now));
  cal_lock.unlock();
  // TODO
}
void PidChain::velocityYawUpdate() {
  // expects were set in function FlightControl::posSubCb()
  // measure were set in function FlightControl::odomCb()
  if (ctrl_enable == false) {
    return;
  }
  cal_lock.lock();
  acc_x.expect = vel_x.update();
  angle_pitch.expect = acc_x.update();
  angle_vel_pitch.expect = angle_pitch.update();
  // angle_vel_pitch.update();

  // angle_vel_yaw.update();

  acc_y.expect = vel_y.update();
  angle_roll.expect = acc_y.update();
  angle_vel_roll.expect = angle_roll.update();
  // angle_vel_roll.update();

  acc_z.expect = vel_z.update();
  thrust.expect = acc_z.update();
  // thrust.update();
  cal_lock.unlock();
}
void PidChain::accelYawUpdate() {
  // expects were set in function FlightControl::posSubCb()
  // measures were set in function FlightControl::odomCb()
  // if (ctrl_enable == false) {
  //   return;
  // }
  cal_lock.lock();

  // angle_pitch.setExpect(acc_x.update());
  // angle_vel_pitch.setExpect(angle_pitch.update());

  // angle_roll.setExpect(acc_y.update());
  // angle_vel_roll.setExpect(angle_roll.update());

  acc_z.setExpect(vel_z.update());
  acc_z.update();
  thrust.setExpect((acc_z.out + acc_z.out_max) / (2 * acc_z.out_max));

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
inline void PidChain::accelSet(double x, double y, double z) {
  cal_lock.lock();
  acc_x.setExpect(x);
  acc_y.setExpect(y);
  acc_z.setExpect(z);
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
inline void PidChain::reset() {
  cal_lock.lock();
  position_x.reset();
  position_y.reset();
  position_z.reset();
  vel_x.reset();
  vel_y.reset();
  vel_z.reset();
  acc_x.reset();
  acc_y.reset();
  thrust.reset();
  angle_pitch.reset();
  angle_yaw.reset();
  angle_roll.reset();
  angle_vel_pitch.reset();
  angle_vel_yaw.reset();
  angle_vel_roll.reset();
  cal_lock.unlock();
}
void PidChain::pubPidDebug() {
  flight_control::PidDebug info;
  cal_lock.lock();
  info.position_expect.x = position_x.expect;
  info.position_expect.y = position_y.expect;
  info.position_expect.z = position_z.expect;
  info.position_now.x = position_x.now;
  info.position_now.y = position_y.now;
  info.position_now.z = position_z.now;
  info.vel_expect.x = vel_x.expect;
  info.vel_expect.y = vel_y.expect;
  info.vel_expect.z = vel_z.expect;
  info.vel_now.x = vel_x.now;
  info.vel_now.y = vel_y.now;
  info.vel_now.z = vel_z.now;
  info.acc_expect.x = acc_x.expect;
  info.acc_expect.y = acc_y.expect;
  info.acc_expect.z = acc_z.expect;
  info.acc_now.x = acc_x.now;
  info.acc_now.y = acc_y.now;
  info.acc_now.z = acc_z.now;
  info.angle_expect.x = angle_roll.expect;
  info.angle_expect.y = angle_pitch.expect;
  info.angle_expect.z = angle_yaw.expect;
  info.angle_now.x = angle_roll.now;
  info.angle_now.y = angle_pitch.now;
  info.angle_now.z = angle_yaw.now;
  info.angle_vel_expect.x = angle_vel_roll.expect;
  info.angle_vel_expect.y = angle_vel_pitch.expect;
  info.angle_vel_expect.z = angle_vel_yaw.expect;
  info.angle_vel_now.x = angle_vel_roll.now;
  info.angle_vel_now.y = angle_vel_pitch.now;
  info.angle_vel_now.z = angle_vel_yaw.now;
  info.thrust_epect = thrust.expect;
  cal_lock.unlock();
  info.header.stamp = ros::Time::now();
  debug_info_pub.publish(info);
}
