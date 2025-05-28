#include "mesh_estimator/ros_io/error_publisher.hpp"

ErrorPublisher::ErrorPublisher(ros::NodeHandle& node) {
  std::string topic_name = "estimation_error";
  _pub = node.advertise<cv_project::estimationError>(topic_name, 32);
}

void ErrorPublisher::pub() {
  _pub.publish(_last_msg);
}

void ErrorPublisher::update(const est_error_t& error) {
  /* TODO */
}

std::shared_ptr<ErrorPublisher> make_error_publisher(ros::NodeHandle& node) {
  return std::make_shared<ErrorPublisher>(node);
}