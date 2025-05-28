#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/estimationError.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

class ErrorPublisher {
public:
  ErrorPublisher(ros::NodeHandle& node);
  void update(const est_error_t& error);
  void pub();

private:
  ros::Publisher _pub;
  cv_project::estimationError _last_msg;
};

std::shared_ptr<ErrorPublisher> make_error_publisher(ros::NodeHandle& node);