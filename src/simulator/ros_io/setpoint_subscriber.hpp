#pragma once

#include "utils/data_struct.hpp"

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <cv_project/setpoint.h>

#include <Eigen/Dense>
#include <memory.h>

class SetpointSubscriber {
public:
  SetpointSubscriber(ros::NodeHandle& node, int agent_ID);
  setpoint_t setpoint();

private:
  setpoint_t _setpoint;
  ros::Subscriber _setpoint_sub;

  int _agent_ID;

  void callback(cv_project::setpoint msg);
};

std::shared_ptr<SetpointSubscriber> make_setpoint_subscriber(
  ros::NodeHandle& node, int agent_ID);