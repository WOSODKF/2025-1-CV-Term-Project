#include "simulator/ros_io/setpoint_subscriber.hpp"

SetpointSubscriber::SetpointSubscriber(ros::NodeHandle& node, int agent_ID)
    : _agent_ID(agent_ID) {
  std::string topic_name =
    "/robot_" + std::to_string(_agent_ID) + "/trajectory_generator/setpoint";

  _setpoint_sub =
    node.subscribe(topic_name, 8, &SetpointSubscriber::callback, this);

  // TODO: initialize _setpoint member
  _setpoint.end_pos = Eigen::Vector3d::Zero();
  _setpoint.end_rot = Eigen::Matrix3d::Identity();
}

setpoint_t SetpointSubscriber::setpoint() {
  return _setpoint;
}

// void SetpointSubscriber::set_control_callback(
//   std::function<void()> control_callback) {
//   _control_callback = std::move(control_callback);
// }

void SetpointSubscriber::callback(cv_project::setpoint msg) {
  _setpoint.end_pos = point_msg_to_eigen(msg.end_pos);

  Eigen::Quaterniond end_quat = quat_msg_to_eigen(msg.end_quat);
  end_quat.normalize();
  _setpoint.end_rot = end_quat.toRotationMatrix();

  _setpoint.end_vel = vector3_msg_to_eigen(msg.end_vel);
  _setpoint.end_w = vector3_msg_to_eigen(msg.end_w);

  // if (_control_callback) {
  //   _control_callback();
  // }
}

std::shared_ptr<SetpointSubscriber> make_setpoint_subscriber(
  ros::NodeHandle& node, int agent_ID) {
  return std::make_shared<SetpointSubscriber>(node, agent_ID);
}