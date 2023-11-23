#include "state_machine.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <nav_msgs/Path.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  StateMachine state_machine(nh);
  ros::spin();
  return 0;
}

StateMachine::StateMachine(ros::NodeHandle &nh) {
  circle_sub = nh.subscribe("/airsim_node/drone_1/circle_poses_gt", 1,
                            &StateMachine::circlePoseCb, this);
  odom_sub = nh.subscribe("/odom_nav", 10, &StateMachine::odomCb, this);
  waypoint_pub =
      nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
  pose_now.header.seq = 0;
}
void StateMachine::circlePoseCb(const airsim_ros::CirclePosesConstPtr &msg) {
  circle_poses.poses = msg->poses;
}
void StateMachine::odomCb(const nav_msgs::OdometryConstPtr &msg) {
#define DEAD_ABS 0.2
  if (abs(msg->pose.pose.position.x -
          circle_poses.poses[waypoint_list[circle_now_index]].position.x) <=
          DEAD_ABS &&
      abs(msg->pose.pose.position.y -
          circle_poses.poses[waypoint_list[circle_now_index]].position.y) <=
          DEAD_ABS &&
      abs(msg->pose.pose.position.z -
          circle_poses.poses[waypoint_list[circle_now_index]].position.z) <=
          DEAD_ABS) {
    ++circle_now_index;
  }
  pose_now = *msg;
}
bool StateMachine::pubNavPath() {
  if (pose_now.header.seq == 0) { // no odom return
    return false;
  }

  nav_msgs::Path waypoint;
  geometry_msgs::PoseStamped pose;

  waypoint.header.frame_id = "world";
  pose.header.frame_id = "world";
  pose.pose.position.x =
      circle_poses.poses[waypoint_list[circle_now_index]].position.x;
  pose.pose.position.y =
      circle_poses.poses[waypoint_list[circle_now_index]].position.y;
  pose.pose.position.z =
      circle_poses.poses[waypoint_list[circle_now_index]].position.z;

  tf2::Quaternion circle_q, now_q;
  now_q.setW(pose_now.pose.pose.orientation.w);
  now_q.setX(pose_now.pose.pose.orientation.x);
  now_q.setY(pose_now.pose.pose.orientation.y);
  now_q.setZ(pose_now.pose.pose.orientation.z);
  double r, p, y;
  tf2::Matrix3x3(now_q).getRPY(r, p, y);  // check yaw direction
  if (abs(y - circle_poses.poses[waypoint_list[circle_now_index]].yaw) >
      M_PI_2) {
    circle_q.setRPY(
        0, 0, circle_poses.poses[waypoint_list[circle_now_index]].yaw + M_PI);
  } else {
    circle_q.setRPY(0, 0,
                    circle_poses.poses[waypoint_list[circle_now_index]].yaw);
  }

  pose.pose.orientation.w = circle_q.w();
  pose.pose.orientation.x = circle_q.x();
  pose.pose.orientation.y = circle_q.y();
  pose.pose.orientation.z = circle_q.z();

  pose.header.stamp = ros::Time::now();
  waypoint.poses.push_back(pose);
  waypoint.header.stamp = ros::Time::now();

  waypoint_pub.publish(waypoint);

  return true;
}
void StateMachine::calFastestWaypoint() {}
