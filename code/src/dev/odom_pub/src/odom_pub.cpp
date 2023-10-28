#include "odom_pub.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_puh");
  ros::NodeHandle nh;
  OdomSubPubNS::OdomSubPub odom_obj;
  odom_obj.init(nh);
  ros::spin();
  return 0;
}

namespace OdomSubPubNS {

void OdomSubPub::init(ros::NodeHandle &nh) {
  odom_sub = nh.subscribe("/airsim_node/drone_1/pose_gt", 5,
                          &OdomSubPub::odomCb, this);
  imu_sub = nh.subscribe("airsim_node/drone_1/imu/imu", 1, &OdomSubPub::imuCb, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
  ROS_INFO("OdomSubPub init success");
}

void OdomSubPub::odomCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  // ROS_INFO("Got odom msg.");
  odom.pose.pose.position.x = msg->pose.position.x;
  odom.pose.pose.position.y = msg->pose.position.y;
  odom.pose.pose.position.z = msg->pose.position.z;
  odom.pose.pose.orientation.w = msg->pose.orientation.w;
  odom.pose.pose.orientation.x = msg->pose.orientation.x;
  odom.pose.pose.orientation.y = msg->pose.orientation.y;
  odom.pose.pose.orientation.z = msg->pose.orientation.z;
  odom.header.seq = msg->header.seq;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "/odom";
  odom.child_frame_id = "/base_link";
  odom_pub.publish(odom);
}

void OdomSubPub::imuCb(const sensor_msgs::ImuConstPtr &msg){
  odom.twist.twist.angular.x = msg->angular_velocity.x;
  odom.twist.twist.angular.y = msg->angular_velocity.y;
  odom.twist.twist.angular.z = msg->angular_velocity.z;
  odom.twist.twist.linear.x = msg->linear_acceleration.x;
  odom.twist.twist.linear.y = msg->linear_acceleration.y;
  odom.twist.twist.linear.z = msg->linear_acceleration.z;
  // odom.twist.covariance = msg->angular_velocity_covariance
}

} // namespace OdomSubPubNS
