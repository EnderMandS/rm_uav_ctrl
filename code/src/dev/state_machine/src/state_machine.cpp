#include "state_machine.h"
#include "airsim_ros/Circle.h"
#include "airsim_ros/CirclePoses.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Core>
#include <cmath>
#include <nav_msgs/Path.h>
#include <ostream>

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
  odom_sub = nh.subscribe("/odom_nav", 1, &StateMachine::odomCb, this);
  waypoint_pub =
      nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
  pose_now.header.seq = 0;
  marker_pub = nh.advertise<visualization_msgs::Marker>("/circle_marker", 17);
  pub_nav_timer =
      nh.createTimer(ros::Duration(5), &StateMachine::pubNavTimerCb, this);
  ROS_INFO("State machine init success.");
}
void StateMachine::circlePoseCb(const airsim_ros::CirclePosesConstPtr &msg) {
  circle_poses.poses = msg->poses;
  for (int i=0; i<circle_poses.poses.size(); ++i) {
    circle_poses.poses[i].position.y = -circle_poses.poses[i].position.y;
    circle_poses.poses[i].position.z = -circle_poses.poses[i].position.z;
  }
  static bool cal_done = false, visual_done = false;
  if (!cal_done) {
    if (calFastestWaypoint()) {
      ROS_INFO("Calculate fastest waypoint success.");
      cal_done = true;
    }
  }
  if (cal_done && !visual_done) {
    if (pubVisualizeCirclePose()) {
      ROS_INFO("Publish visualizetion of circle poses success.");
      visual_done = true;
    }
  }
}
void StateMachine::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  if (circle_poses.poses.empty()) {
    return;
  }

#define DEAD_ABS 0.2
  if (abs(msg->pose.pose.position.x -
          circle_poses.poses[waypoint_list[circle_now_index]].position.x) <=
          DEAD_ABS &&
      abs(msg->pose.pose.position.y -
          circle_poses.poses[waypoint_list[circle_now_index]].position.y) <=
          DEAD_ABS &&
      abs(msg->pose.pose.position.z -
          circle_poses.poses[waypoint_list[circle_now_index]].position.z) <=
          DEAD_ABS &&
      circle_now_index <
          (sizeof(waypoint_list) / sizeof(waypoint_list[0]) - 1)) {
    if (pubNavPath(circle_now_index + 1)) {
      ++circle_now_index;
    }
  }
  pose_now = *msg;
}
void StateMachine::pubNavTimerCb(const ros::TimerEvent &e) {
  static bool first_pub = false;
  // if (!first_pub) {
  //   first_pub = pubNavPath(0);
  // }
}
bool StateMachine::pubNavPath(int circle_index) {
  if (pose_now.header.seq == 0) { // no odom return
    ROS_INFO("No odom message. Wait to publish nav path.");
    return false;
  }
  if (waypoint_pub.getNumSubscribers() < 1) {
    ROS_WARN("No subscriber on topic /waypoint_generator/waypoints, publish "
             "nav path fail.");
    return false;
  }

  nav_msgs::Path waypoint;
  geometry_msgs::PoseStamped pose;

  waypoint.header.frame_id = "world";
  pose.header.frame_id = "world";
  pose.pose.position.x =
      circle_poses.poses[waypoint_list[circle_index]].position.x;
  pose.pose.position.y =
      circle_poses.poses[waypoint_list[circle_index]].position.y;
  pose.pose.position.z =
      circle_poses.poses[waypoint_list[circle_index]].position.z;

  tf2::Quaternion circle_q, now_q;
  now_q.setW(pose_now.pose.pose.orientation.w);
  now_q.setX(pose_now.pose.pose.orientation.x);
  now_q.setY(pose_now.pose.pose.orientation.y);
  now_q.setZ(pose_now.pose.pose.orientation.z);
  double r, p, y;
  tf2::Matrix3x3(now_q).getRPY(r, p, y); // check yaw direction
  if (abs(y - circle_poses.poses[waypoint_list[circle_index]].yaw) > M_PI_2) {
    circle_q.setRPY(0, 0,
                    circle_poses.poses[waypoint_list[circle_index]].yaw + M_PI);
  } else {
    circle_q.setRPY(0, 0, circle_poses.poses[waypoint_list[circle_index]].yaw);
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
float pointDistance(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2) {
  return std::sqrt(pow(x1.x - x2.x, 2) + pow(x1.y - x2.y, 2) +
                   pow(x1.z - x2.z, 2));
}
bool StateMachine::calFastestWaypoint() {
  if (circle_poses.poses.empty()) {
    ROS_ERROR("circle poses vector empty!");
    return false;
  } else if (circle_poses.poses.size() != 17) {
    ROS_ERROR("circle poses vector size not equal to 17!");
    return false;
  }

  // 0~4 total 5 static
  // 5~11 total 7 dynamic
  // 12~14 total 3 static
  // 15 total 1 avoid wing static
  // 16 total 1 dynamic
  airsim_ros::Circle pass_circle[9];
  pass_circle[0] = circle_poses.poses[4];
  pass_circle[8] = circle_poses.poses[12];
  for (int i = 0; i < 7; ++i) {
    pass_circle[1 + i] = circle_poses.poses[5 + i];
  }

  // calculate distance
  Eigen::Matrix<float, 9, 9> distance;
  distance.setZero();
  for (int i = 0; i < 9 - 1; ++i) {
    for (int j = i + 1; j < 9; ++j) {
      distance(i, j) =
          pointDistance(pass_circle[i].position, pass_circle[j].position);
    }
  }
  distance(0, 8) = 999.f;
  // copy symmetry matrix
  distance += distance.transpose().eval();
  std::cout << "Distance matrix is :\n" << distance << std::endl;

  // find min distance
  float min_distance = 9999.f;
  for (int i = 1; i < 8; ++i) {
    float now_distance = distance(0, i);
    for (int j = 1; j < 8; ++j) {
      if (i != j) {
        now_distance += distance(i, j);
        for (int k = 1; k < 8; ++k) {
          if (j != k && k != i) {
            now_distance += distance(j, k) + distance(k, 8);
            if (now_distance <= min_distance) {
              min_distance = now_distance;
              waypoint_list[5] = i + 4;
              waypoint_list[6] = j + 4;
              waypoint_list[7] = k + 4;
            }
          }
        }
      }
    }
  }
  ROS_INFO("Min index: %d, %d, %d. Min distance: %f", waypoint_list[5],
           waypoint_list[6], waypoint_list[7], min_distance);

  return true;
}
bool StateMachine::pubVisualizeCirclePose() {
  if (marker_pub.getNumSubscribers() < 1) {
    // ROS_INFO("Wait for rviz to sub marker.");
    return false;
  }
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.ns = "circle_poses";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.scale.x = 1.0;
  marker.scale.y = 0.35;
  marker.scale.z = 0.35;

  tf2::Quaternion q;
  for (int i = 0; i < circle_poses.poses.size(); ++i) {
    marker.pose.position.x = circle_poses.poses[i].position.x;
    marker.pose.position.y = circle_poses.poses[i].position.y;
    marker.pose.position.z = circle_poses.poses[i].position.z;
    q.setRPY(0, 0, circle_poses.poses[i].yaw);
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    if (i >= 5 && i <= 11) {
      // 7 choose 3
      if (i == waypoint_list[5] || i == waypoint_list[6] ||
          i == waypoint_list[7]) {
        // cal min distance circle
        marker.color.r = 0.f;
        marker.color.g = 0.f;
        marker.color.b = 1.f;
        marker.color.a = 0.9f;
      } else {
        // will not pass
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        marker.color.a = 0.9f;
      }
    } else {
      // must pass
      marker.color.r = 1.f;
      marker.color.g = 0.f;
      marker.color.b = 0.f;
      marker.color.a = 0.9f;
    }
    marker.id = i;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
  }
  return true;
}
