#ifndef _STATE_MACHINE_
#define _STATE_MACHINE_

#include "nav_msgs/Odometry.h"
#include <airsim_ros/CirclePoses.h>
#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/Point.h>
#include <vector>

struct FlyPose {
  float position_x;
  float position_y;
  float position_z;
  float yaw;
};
struct Circle3Pose {
  FlyPose before, mid, after;
};

class StateMachine {
public:
  StateMachine(ros::NodeHandle &);

private:
  ros::Subscriber circle_sub, odom_sub;
  ros::Publisher waypoint_pub, marker_pub;
  ros::Timer pub_waypoint_timer;

  enum FlyState{
    UP, SMOOTH, DOWN, CROSS, STOP
  };

  bool init_cal = false;
  int waypoint_now = 0;
  int fly_state=FlyState::STOP;
  int waypoint_list[13]={0,1,2,3,4,5,6,7,12,13,14,15,16};
  std::vector<FlyPose> v_pose;
  airsim_ros::CirclePoses circle_poses;
  nav_msgs::Odometry pose_now;
  unsigned char trajectory_flag=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
  float fly_height=50.f;

  void circlePoseCb(const airsim_ros::CirclePosesConstPtr &);
  void odomCb(const nav_msgs::OdometryConstPtr &);
  void pubWayPointTimerCb(const ros::TimerEvent &);
  void pubFsmCmdCb(const ros::TimerEvent &);
  bool pubNavPath(int);
  bool calFastestWaypoint();
  bool calTopHeight();
  bool calAllWaypoint();
  bool pubVisualizeCirclePose();
};

#endif