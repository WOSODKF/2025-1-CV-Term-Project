#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/robotState.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>


class StatePublisher {
public:
  StatePublisher(ros::NodeHandle& node, int agent_ID);
  void pub();

  void update_state(const robot_state_t& robot_state);

private:
  ros::Publisher _data_pub;
  int _agent_ID;

  cv_project::robotState _last_state_msg;
};

std::shared_ptr<StatePublisher> make_state_publisher(
  ros::NodeHandle node, int agent_ID);