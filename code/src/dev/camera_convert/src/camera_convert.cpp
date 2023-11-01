#include "camera_convert.h"
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_convert");
  ros::NodeHandle nh;
  auto camera_converter = CameraConvert(nh);
  ROS_INFO("Camera convert start.");
  ros::spin();
  return 0;
}

CameraConvert::CameraConvert(ros::NodeHandle &nh) : it(nh) {
  pub_depth = it.advertise("/camera/depth", 5);
  pub_scene = it.advertise("/camera/scens", 5);
  sub_depth = it.subscribe("/airsim_node/drone_1/front_center/DepthPlanar", 5,
                           &CameraConvert::depthCb, this);
  sub_scene = it.subscribe("/airsim_node/drone_1/front_center/Scene", 5,
                           &CameraConvert::sceneCb, this);

  pub_cam_info =
      nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 5);
  sub_cam_info =
      nh.subscribe("/airsim_node/drone_1/front_center/DepthPlanar/camera_info",
                   5, &CameraConvert::cam_infoCb, this);
}
void CameraConvert::cam_infoCb(const sensor_msgs::CameraInfoPtr &msg) {
  std::shared_ptr<sensor_msgs::CameraInfo> msg_copy(
      new sensor_msgs::CameraInfo(*msg));
  msg_copy->header.frame_id = "camera";
  msg_copy->header.stamp = msg->header.stamp;
  pub_cam_info.publish(*msg_copy);
}
void CameraConvert::depthCb(const sensor_msgs::ImageConstPtr &msg) {
  auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  std::shared_ptr<std_msgs::Header> header(new std_msgs::Header());
  header->frame_id = "camera";
  header->stamp = msg->header.stamp;
  auto msg_copy =
      cv_bridge::CvImage(*header, cv_ptr->encoding, cv_ptr->image).toImageMsg();
  pub_depth.publish(*msg_copy);
}
void CameraConvert::sceneCb(const sensor_msgs::ImageConstPtr &msg) {
  auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  std::shared_ptr<std_msgs::Header> header(new std_msgs::Header());
  header->frame_id = "camera";
  header->stamp = msg->header.stamp;
  auto msg_copy =
      cv_bridge::CvImage(*header, cv_ptr->encoding, cv_ptr->image).toImageMsg();
  pub_scene.publish(*msg_copy);
}
