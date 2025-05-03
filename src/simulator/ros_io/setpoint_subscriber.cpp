#include "simulator/ros_io/setpoint_subscriber.hpp"

SetpointSubscriber::SetpointSubscriber(ros::NodeHandle& node, int agent_ID)
    : _agent_ID(agent_ID) {
  std::string topic_name =
    "/robot_" + std::to_string(_agent_ID) + "/trajectory_generator/setpoint";
    
  _setpoint_sub =
    node.subscribe(topic_name, 8, &SetpointSubscriber::callback, this);

  // TODO: initialize _setpoint member
}

setpoint_t SetpointSubscriber::setpoint() {
  return _setpoint;
}

void SetpointSubscriber::callback(cv_project::setpoint msg) {
  // TODO: assign value to _setpoint
}

std::shared_ptr<SetpointSubscriber> make_setpoint_subscriber(
  ros::NodeHandle& node, int agent_ID) {
  return std::make_shared<SetpointSubscriber>(node, agent_ID);
}