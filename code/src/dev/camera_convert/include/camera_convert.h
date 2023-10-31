#ifndef _CAMERA_CONVERT_H_
#define _CAMERA_CONVERT_H_

#include "image_transport/image_transport.h"
#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/CameraInfo.h"
#include <ros/ros.h>

class CameraConvert {
public:
  CameraConvert(ros::NodeHandle &);

private:
  image_transport::ImageTransport it;
  image_transport::Publisher pub_depth, pub_scene;
  image_transport::Subscriber sub_depth, sub_scene;
  void depthCb(const sensor_msgs::ImageConstPtr &);
  void sceneCb(const sensor_msgs::ImageConstPtr &);

  ros::Publisher pub_cam_info;
  ros::Subscriber sub_cam_info;
  sensor_msgs::CameraInfo cam_info;
  void cam_infoCb(const sensor_msgs::CameraInfoPtr &);
};

#endif // _CAMERA_CONVERT_H_