#include "mesh_estimator/ros_io/measurement_subscriber.hpp"

MeasurementSubscriber::MeasurementSubscriber(ros::NodeHandle& node, int agent_ID)
    : _agent_ID(agent_ID) {
  std::string topic_name = "/simulator/measurement_" + std::to_string(agent_ID);
  _sub = node.subscribe(topic_name, 8, &MeasurementSubscriber::callback, this);
}

robot_measurement_t MeasurementSubscriber::get_data() {
  return _last_measurement_data;
}

void MeasurementSubscriber::callback(cv_project::robotMeasurement msg) {
  _last_measurement_data.update(msg);
}

std::shared_ptr<MeasurementSubscriber> make_measurement_subscriber(
  ros::NodeHandle& node, int agent_ID) {
  return std::make_shared<MeasurementSubscriber>(node, agent_ID);
}