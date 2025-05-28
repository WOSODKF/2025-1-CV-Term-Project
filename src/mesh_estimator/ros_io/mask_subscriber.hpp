#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/clothMask.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

class MaskSubscriber {
public:
  MaskSubscriber(ros::NodeHandle& node, int agent_ID);
  mask_data_t get_data();

private:
  int _agent_ID;

  ros::Subscriber _sub;
  cv_project::clothMask _last_msg;
  mask_data_t _last_mask_data;

  void callback(cv_project::clothMask msg);
};

std::shared_ptr<MaskSubscriber> make_mask_subscriber(ros::NodeHandle& node, int agent_ID);