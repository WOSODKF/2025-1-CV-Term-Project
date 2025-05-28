#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/estimatedParam.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

class ParamPublisher{
public:
  ParamPublisher(ros::NodeHandle& node);
  void update(const est_mesh_param_t& param);
  void pub();

private:
  ros::Publisher _pub;
  cv_project::estimatedParam _last_msg;
};

std::shared_ptr<ParamPublisher> make_param_publisher(ros::NodeHandle& node);