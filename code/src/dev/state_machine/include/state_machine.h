#ifndef _STATE_MACHINE_
#define _STATE_MACHINE_

#include "nav_msgs/Odometry.h"
#include <airsim_ros/CirclePoses.h>
#include <ros/ros.h>

class StateMachine {
public:
  StateMachine(ros::NodeHandle &);

private:
  ros::Subscriber circle_sub, odom_sub;
  ros::Publisher waypoint_pub, marker_pub;
  ros::Timer pub_nav_timer;

  int circle_now_index = 0;
  int waypoint_list[13]={0,1,2,3,4,5,6,7,12,13,14,15,16};
  airsim_ros::CirclePoses circle_poses;
  nav_msgs::Odometry pose_now;

  void circlePoseCb(const airsim_ros::CirclePosesConstPtr &);
  void odomCb(const nav_msgs::OdometryConstPtr &);
  void pubNavTimerCb(const ros::TimerEvent &);
  bool pubNavPath(int);
  bool calFastestWaypoint();
  bool pubVisualizeCirclePose();
};

#endif