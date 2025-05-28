#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/estimationError.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

class ErrorSubscriber {
public:
  ErrorSubscriber(ros::NodeHandle& node);
  est_error_t get_data();

private:
  ros::Subscriber _sub;
  cv_project::estimationError _last_msg;
  est_error_t _last_error_data;

  void callback(cv_project::estimationError msg);
};

std::shared_ptr<ErrorSubscriber> make_error_subscriber(ros::NodeHandle& node);