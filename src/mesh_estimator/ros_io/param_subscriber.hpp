#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/estimatedParam.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

class ParamSubscriber {
public:
  ParamSubscriber(ros::NodeHandle& node);
  est_mesh_param_t get_data();

private:
  ros::Subscriber _sub;
  cv_project::estimatedParam _last_msg;
  est_mesh_param_t _last_param_data;

  void callback(cv_project::estimatedParam msg);
};

std::shared_ptr<ParamSubscriber> make_param_subscriber(ros::NodeHandle& node);