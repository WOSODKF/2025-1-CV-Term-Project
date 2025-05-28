#include "simulator/ros_io/state_publisher.hpp"

StatePublisher::StatePublisher(ros::NodeHandle& node, int agent_ID) {
  std::string topic_name = "state_" + std::to_string(agent_ID);
  _data_pub = node.advertise<cv_project::robotState>(topic_name, 32);
  // TODO: initialize _last_state_msg
}

void StatePublisher::pub() {
  // std::cout << "Pub called" << std::endl << _last_state_msg << std::endl;
  _data_pub.publish(_last_state_msg);
}

void StatePublisher::update_state(const robot_state_t& robot_state) {
  // TODO: convert robot_data into _last_state_msg
  auto time = ros::Time(robot_state.t);
  _last_state_msg.header.stamp = time;
  _last_state_msg.header.frame_id = "robot_" + std::to_string(_agent_ID);

  _last_state_msg.end_pos = eigen_to_point_msg(robot_state.end_pos);
  auto end_quat = Quaterniond(robot_state.end_rot);
  end_quat.normalize();
  _last_state_msg.end_quat = eigen_to_quat_msg(end_quat);

  _last_state_msg.cam_pos = eigen_to_point_msg(robot_state.cam_pos);
  auto cam_quat = Quaterniond(robot_state.cam_rot);
  cam_quat.normalize();
  _last_state_msg.cam_quat = eigen_to_quat_msg(cam_quat);

  _last_state_msg.joint_pos_0 = robot_state.joint_pos_0;
  _last_state_msg.joint_pos_1 = robot_state.joint_pos_1;
  _last_state_msg.joint_pos_2 = robot_state.joint_pos_2;
  _last_state_msg.joint_pos_3 = robot_state.joint_pos_3;
  _last_state_msg.joint_pos_4 = robot_state.joint_pos_4;
  _last_state_msg.joint_pos_5 = robot_state.joint_pos_5;
}

std::shared_ptr<StatePublisher> make_state_publisher(
  ros::NodeHandle node, int agent_ID) {
  return std::make_shared<StatePublisher>(node, agent_ID);
}