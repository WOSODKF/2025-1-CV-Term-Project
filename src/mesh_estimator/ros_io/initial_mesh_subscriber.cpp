#include "mesh_estimator/ros_io/initial_mesh_subscriber.hpp"

InitialMeshSubscriber::InitialMeshSubscriber(
  ros::NodeHandle& node, std::shared_ptr<config_t> config) {
  std::string topic_name = "/simulator/mesh_GT";
  _sub = node.subscribe(topic_name, 3, &InitialMeshSubscriber::callback, this);
  _mesh_data.init(-1, config->mesh.rows, config->mesh.cols);
  _mesh_ready = false;
}

void InitialMeshSubscriber::callback(cv_project::mesh msg) {
  _mesh_data.t = msg.header.stamp.toSec();
  for (int i = 0; i < msg.cols * msg.rows; i++) {
    _mesh_data.points[i] = point_msg_to_eigen(msg.positions[i]);
  }
  _mesh_ready = true;
  _sub.shutdown();
}

bool InitialMeshSubscriber::initial_mesh_ready(){
  return _mesh_ready;
}

mesh_data_t InitialMeshSubscriber::get_data() {
  return _mesh_data;
}

std::shared_ptr<InitialMeshSubscriber> make_initial_mesh_subscriber(
  ros::NodeHandle& node, std::shared_ptr<config_t> config){
  return std::make_shared<InitialMeshSubscriber>(node, config);
}