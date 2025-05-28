#include "utils/mesh_publisher.hpp"

MeshPublisher::MeshPublisher(ros::NodeHandle& node, const std::string& name) {
  std::string topic_name = "mesh_" + name;
  _pub = node.advertise<cv_project::mesh>(topic_name, 32);
}

void MeshPublisher::pub() {
//   _pub.publish(_last_msg);
}

void MeshPublisher::update(const mesh_data_t& mesh) {
  /* TODO */
}

std::shared_ptr<MeshPublisher> make_error_publisher(
  ros::NodeHandle& node, const std::string& name) {
  return std::make_shared<MeshPublisher>(node, name);
}