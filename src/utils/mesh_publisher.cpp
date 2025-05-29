#include "utils/mesh_publisher.hpp"

MeshPublisher::MeshPublisher(
  ros::NodeHandle& node, const std::string& name,
  const mesh_data_t& mesh_info) {
  std::string topic_name = "mesh_" + name;
  _pub = node.advertise<cv_project::mesh>(topic_name, 32);

  _last_msg.rows = mesh_info.rows;
  _last_msg.cols = mesh_info.cols;
  _last_msg.positions.resize(_last_msg.rows * _last_msg.cols);
}

void MeshPublisher::pub() {
    _pub.publish(_last_msg);
}

void MeshPublisher::update(const mesh_data_t& mesh, bool initial) {
  _last_msg.header.stamp = ros::Time(mesh.t);
  _last_msg.header.frame_id = "mesh";

  _last_msg.initial = initial;

  _last_msg.rows = mesh.rows;
  _last_msg.cols = mesh.cols;

  for (int i = 0; i < mesh.rows; i++) {
    for (int j = 0; j < mesh.cols; j++) {
      geometry_msgs::Point point;
      point.x = mesh.points[j + mesh.cols * i](0);
      point.y = mesh.points[j + mesh.cols * i](1);
      point.z = mesh.points[j + mesh.cols * i](2);

      _last_msg.positions[j + mesh.cols * i] = point;
    }
  }
}

std::shared_ptr<MeshPublisher> make_mesh_publisher(
  ros::NodeHandle& node, const std::string& name,
  const mesh_data_t& mesh_info) {
  return std::make_shared<MeshPublisher>(node, name, mesh_info);
}