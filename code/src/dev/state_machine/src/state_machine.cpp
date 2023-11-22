#include "state_machine.h"

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
}

void StateMachine::circlePoseCb(const airsim_ros::CirclePosesConstPtr &msg) {
  circle_poses = *msg;
  waypoint_pub.publish(msg->poses[waypoint_list[circle_now_index]]);
}
void StateMachine::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  
}
