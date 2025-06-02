#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/robotMeasurement.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <functional>

class MeasurementSubscriber {
public:
  MeasurementSubscriber(ros::NodeHandle& node, int agent_ID);
  robot_measurement_t get_data();

  void set_flag_callback(std::function<void(const std::string&)> flag_callback);

private:
  int _agent_ID;

  ros::Subscriber _sub;
  cv_project::robotMeasurement _last_msg;
  robot_measurement_t _last_measurement_data;

  std::function<void(const std::string&)> _flag_callback;

  void callback(cv_project::robotMeasurement msg);
};

std::shared_ptr<MeasurementSubscriber> make_measurement_subscriber(
  ros::NodeHandle& node, int agent_ID);