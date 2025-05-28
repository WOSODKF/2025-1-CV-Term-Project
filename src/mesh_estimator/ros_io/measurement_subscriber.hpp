#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/robotMeasurement.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

class MeasurementSubscriber {
public:
  MeasurementSubscriber(ros::NodeHandle& node, int agent_ID);
  robot_measurement_t get_data();

private:
  int _agent_ID;

  ros::Subscriber _sub;
  cv_project::robotMeasurement _last_msg;
  robot_measurement_t _last_measurement_data;

  void callback(cv_project::robotMeasurement msg);
};

std::shared_ptr<MeasurementSubscriber> make_measurment_subscriber(
  ros::NodeHandle& node, int agent_ID);