#pragma once

#include "utils/data_struct.hpp"

#include <cv_project/mesh.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

// publisher for global usage(GT mesh, estimated mesh)
class MeshPublisher {
public:
  MeshPublisher(
    ros::NodeHandle& node, const std::string& name,
    const mesh_data_t& mesh_info);
  void update(const mesh_data_t& mesh, bool initial);
  void pub();

private:
  ros::Publisher _pub;
  cv_project::mesh _last_msg;
};

std::shared_ptr<MeshPublisher> make_mesh_publisher(
  ros::NodeHandle& node, const std::string& name, const mesh_data_t& mesh_info);