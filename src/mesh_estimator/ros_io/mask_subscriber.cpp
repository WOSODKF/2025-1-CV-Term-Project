#include "mesh_estimator/ros_io/mask_subscriber.hpp"

MaskSubscriber::MaskSubscriber(
  ros::NodeHandle& node, std::shared_ptr<config_t> config, int agent_ID)
    : _agent_ID(agent_ID) {
  std::string topic_name = "/segmentor/cloth_mask_" + std::to_string(agent_ID);
  _sub = node.subscribe(topic_name, 3, &MaskSubscriber::callback, this);

  _last_mask_data.init(config);
}

mask_data_t MaskSubscriber::get_data() {
  return _last_mask_data;
}

void MaskSubscriber::callback(cv_project::clothMask msg) {
  _last_mask_data.update(msg);
}

std::shared_ptr<MaskSubscriber> make_mask_subscriber(
  ros::NodeHandle& node, std::shared_ptr<config_t> config, int agent_ID) {
  return std::make_shared<MaskSubscriber>(node, config, agent_ID);
}