#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/robotMeasurement.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>

class MeasurementPublisher {
public:
  MeasurementPublisher(ros::NodeHandle& node, int agent_ID, int cam_ID);
  void pub_view();
  void pub_measurement();
  void update_measurement(const robot_state_t& state, const mujoco_robot_wrench_t& robot_wrench);
  void update_view(const cv::Mat& bgr_img, double time);

private:
  ros::Publisher _view_pub;
  ros::Publisher _measurement_pub;

  int _agent_ID;
  int _cam_ID;

  sensor_msgs::ImagePtr _last_view_msg;
  cv_project::robotMeasurement _last_measurement_msg;  
};

std::shared_ptr<MeasurementPublisher> make_measurement_publisher(
  ros::NodeHandle node, int agent_ID, int cam_ID);