#include "simulator/ros_io/data_publisher.hpp"

DataPublisher::DataPublisher(ros::NodeHandle& node, int agent_ID){
  _data_pub = node.advertise<cv_project::robotData>("robotData", 32);
  // TODO: initialize _last_data_msg
}

void DataPublisher::pub() {
  _data_pub.publish(_last_data_msg);
}

void DataPublisher::update_data(const mujoco_robot_data_t& robot_data) {
  // TODO: convert robot_data into _last_data_msg
}

std::shared_ptr<DataPublisher> make_data_publisher(
  ros::NodeHandle node, int agent_ID) {
  return std::make_shared<DataPublisher>(node, agent_ID);
}