#include "param_estimator/ros_io/param_publisher.hpp"

ParamPublisher::ParamPublisher(ros::NodeHandle& node){
  std::string topic_name = "estimated_parameters";
  _pub = node.advertise<cv_project::estimatedParam>(topic_name, 32);
}

void ParamPublisher::pub(){
  _pub.publish(_last_msg);
}

void ParamPublisher::update(const est_param_t& param){
    /* TODO */
}

std::shared_ptr<ParamPublisher> make_param_publisher(ros::NodeHandle& node){
  return std::make_shared<ParamPublisher>(node);
}