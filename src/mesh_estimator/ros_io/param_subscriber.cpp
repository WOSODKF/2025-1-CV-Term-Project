#include "mesh_estimator/ros_io/param_subscriber.hpp"

ParamSubscriber::ParamSubscriber(ros::NodeHandle& node) {
  std::string topic_name = "estimated_parameters";
  _sub = node.subscribe(topic_name, 3, &ParamSubscriber::callback, this);
}

est_param_t ParamSubscriber::get_data() {
  return _last_param_data;
}

void ParamSubscriber::callback(cv_project::estimatedParam msg) {
  /*TODO: _last_mask_data <- msg*/
}

std::shared_ptr<ParamSubscriber> make_param_subscriber(ros::NodeHandle& node) {
  return std::make_shared<ParamSubscriber>(node);
}