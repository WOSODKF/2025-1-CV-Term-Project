#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/robotWrench.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>

class MeasurementPublisher {
public:
  MeasurementPublisher(ros::NodeHandle& node, int agent_ID);
  void pub();

  void update_wrench(const mujoco_robot_wrench_t& robot_wrench);
  void update_view(const cv::Mat& rgb_img);

private:
  ros::Publisher _wrench_pub;
  ros::Publisher _view_pub;
  int _agent_ID;  // = cam ID

  cv_project::robotWrench _last_wrench_msg;
  sensor_msgs::ImagePtr _last_view_msg;
};

std::shared_ptr<MeasurementPublisher> make_measurement_publisher(
  ros::NodeHandle node, int agent_ID);