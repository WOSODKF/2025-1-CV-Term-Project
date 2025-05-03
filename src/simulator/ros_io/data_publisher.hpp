#pragma once

#include <cv_project/robotData.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "utils/data_struct.hpp"

class DataPublisher {
public:
  DataPublisher(ros::NodeHandle& node, int agent_ID);
  void pub();

  void update_data(const mujoco_robot_data_t& robot_data);

private:
  ros::Publisher _data_pub;
  int _agent_ID;

  cv_project::robotData _last_data_msg;
};

std::shared_ptr<DataPublisher> make_data_publisher(
  ros::NodeHandle node, int agent_ID);