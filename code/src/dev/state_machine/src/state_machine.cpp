#include "state_machine.h"
#include "airsim_ros/Circle.h"
#include "geometry_msgs/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Core>
#include <cmath>
#include <nav_msgs/Path.h>
#include <ostream>
#include <std_srvs/Empty.h>

#define VISUAL

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  StateMachine state_machine(nh);
  // ros::Duration(2.0).sleep();
  ros::spin();
  return 0;
}

StateMachine::StateMachine(ros::NodeHandle &nh) {
  circle_sub = nh.subscribe("/airsim_node/drone_1/circle_poses_gt", 1,
                            &StateMachine::circlePoseCb, this);
  odom_sub = nh.subscribe("/odom_nav", 1, &StateMachine::odomCb, this);
  waypoint_pub =
      nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 5);
  pose_now.header.seq = 0;
  marker_pub = nh.advertise<visualization_msgs::Marker>("/circle_marker", 100);
  pub_waypoint_timer =
      nh.createTimer(ros::Duration(2), &StateMachine::pubWayPointTimerCb, this);
  ROS_INFO("State machine init success.");
}
void StateMachine::circlePoseCb(const airsim_ros::CirclePosesConstPtr &msg) {
  circle_poses.poses = msg->poses;
  for (int i = 0; i < circle_poses.poses.size(); ++i) {
    circle_poses.poses[i].position.y = -circle_poses.poses[i].position.y;
    circle_poses.poses[i].position.z = -circle_poses.poses[i].position.z;
  }
  if (!init_cal) {
    if (calFastestWaypoint() && calTopHeight() && calAllWaypoint()) {
      ROS_INFO("Calculate fastest waypoint, height and all waypoints success.");
      init_cal = true;
      trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EXEC;
    }
  }
#ifdef VISUAL
  static bool visual_done = false;
  if (init_cal && !visual_done) {
    if (pubVisualizeCirclePose()) {
      ROS_INFO("Publish visualizetion of circle poses success.");
      visual_done = true;
    }
  }
#endif
}
void StateMachine::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  if (circle_poses.poses.empty() || init_cal==false) {
    return;
  }

  double distance = std::sqrt(
      pow(v_pose[waypoint_now].position_x - msg->pose.pose.position.x, 2) +
      pow(v_pose[waypoint_now].position_y - msg->pose.pose.position.y, 2) +
      pow(v_pose[waypoint_now].position_z - msg->pose.pose.position.z, 2));
  if (distance < 0.25) {
    if (waypoint_now < v_pose.size()-1) {
      if (pubNavPath(waypoint_now+1)) {
        ++waypoint_now;
      }
    }
    else {
      ROS_INFO("All waypoint have published.");
      trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    }
  }

  pose_now = *msg;
}
void StateMachine::pubWayPointTimerCb(const ros::TimerEvent &e) {
  pubNavPath(waypoint_now);
}
bool StateMachine::pubNavPath(int index) {
  if (pose_now.header.seq == 0) { // no odom return
    ROS_INFO("StateMachine No odom message. Waiting.");
    return false;
  }
  if (waypoint_pub.getNumSubscribers() < 1) {
    ROS_WARN("StateMachine No subscriber on topic "
             "/planning/pos_cmd, publish "
             "waypoint path fail.");
    return false;
  }
  if (init_cal == false) {
    ROS_INFO("Init calculate not complete, waiting.");
    return false;
  }

  quadrotor_msgs::PositionCommand waypoint;
  waypoint.header.frame_id = "world";
  waypoint.trajectory_flag = this->trajectory_flag;
  waypoint.position.x = v_pose[index].position_x;
  waypoint.position.y = v_pose[index].position_y;
  waypoint.position.z = v_pose[index].position_z;
  waypoint.yaw = v_pose[index].yaw;
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

  marker.ns = "waypoint";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.color.r = 1.f;
  marker.color.g = 0.f;
  marker.color.b = 0.f;
  marker.color.a = 0.9f;
  for (int i=0; i<v_pose.size(); ++i) {
    marker.id = i;
    marker.pose.position.x = v_pose[i].position_x;
    marker.pose.position.y = v_pose[i].position_y;
    marker.pose.position.z = v_pose[i].position_z;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
  }

  return true;
}
bool StateMachine::calTopHeight() {
  if (circle_poses.poses.empty()) {
    ROS_ERROR("circle poses vector empty!");
    return false;
  } else if (circle_poses.poses.size() != 17) {
    ROS_ERROR("circle poses vector size not equal to 17!");
    return false;
  }

  float height_max = 0.f;
  for (int i = 0; i < circle_poses.poses.size(); ++i) {
    if (circle_poses.poses[i].position.z > height_max) {
      height_max = circle_poses.poses[i].position.z;
    }
  }
  if (height_max == 0.f) {
    return false;
  }
  fly_height = height_max + 10;
  ROS_INFO("Circle Max height: %f", height_max);
  ROS_INFO("Fly Max height: %f", fly_height);

  return true;
}
bool StateMachine::calAllWaypoint() {
  for (int i = 0; i < 13; ++i) {
    Circle3Pose circle;
    circle.mid.position_x = circle_poses.poses[waypoint_list[i]].position.x;
    circle.mid.position_y = circle_poses.poses[waypoint_list[i]].position.y;
    circle.after.position_z = circle.before.position_z = circle.mid.position_z =
        circle_poses.poses[waypoint_list[i]].position.z;
    circle.mid.yaw = circle_poses.poses[waypoint_list[i]].yaw;
    // double cross_yaw;
    // if (i == 0) {
    //   cross_yaw = atan2(circle.mid.position_y, circle.mid.position_x);
    // } else {
    //   cross_yaw = atan2(circle.mid.position_y - v_pose.end()->position_y,
    //                     circle.mid.position_x - v_pose.end()->position_x);
    // }
    // if (abs(cross_yaw - circle.mid.yaw) > M_PI_2) {
    //   circle.before.yaw = circle.mid.yaw + M_PI;
    //   if (circle.before.yaw > M_PI) {
    //     circle.before.yaw -= 2 * M_PI;
    //   } else if (circle.before.yaw < -M_PI) {
    //     circle.before.yaw += 2 * M_PI;
    //   }
    // }

    circle.after.yaw = circle.before.yaw = circle.mid.yaw;
    circle.after.position_x = circle.mid.position_x + cos(circle.after.yaw);
    circle.after.position_y = circle.mid.position_y + sin(circle.after.yaw);
    circle.before.position_x =
        circle.mid.position_x + cos(circle.after.yaw + M_PI);
    circle.before.position_y =
        circle.mid.position_y + cos(circle.after.yaw + M_PI);
    FlyPose pos;
    pos.position_z = fly_height;
    if (i == 0) {
      pos.position_x = 0;
      pos.position_y = 0;
      pos.yaw = atan2(circle.before.position_y, circle.before.position_x);
    } else {
      pos.position_x = v_pose.end()->position_x;
      pos.position_y = v_pose.end()->position_y;
      pos.yaw = atan2(circle.before.position_y - pos.position_y,
                      circle.before.position_x - pos.position_x);
    }
    v_pose.push_back(pos);
    pos.position_x = circle.before.position_x;
    pos.position_y = circle.before.position_y;
    v_pose.push_back(pos);
    pos.position_z = circle.before.position_z;
    pos.yaw = circle.before.yaw;
    v_pose.push_back(pos);
    pos.position_x = circle.after.position_x;
    pos.position_y = circle.after.position_y;
    v_pose.push_back(pos);
  }
  ROS_INFO("All waypoint vector size: %d", (int)v_pose.size());
  return true;
}
