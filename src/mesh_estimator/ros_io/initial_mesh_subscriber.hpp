#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/mesh.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <functional>

class InitialMeshSubscriber {
public:
  InitialMeshSubscriber(
    ros::NodeHandle& node, std::shared_ptr<config_t> config);
  bool initial_mesh_ready();
  mesh_data_t get_data();

private:
  bool _mesh_ready;

  ros::Subscriber _sub;
  cv_project::mesh _last_msg;
  mesh_data_t _mesh_data;

  void callback(cv_project::mesh msg);
};

std::shared_ptr<InitialMeshSubscriber> make_initial_mesh_subscriber(
  ros::NodeHandle& node, std::shared_ptr<config_t> config);