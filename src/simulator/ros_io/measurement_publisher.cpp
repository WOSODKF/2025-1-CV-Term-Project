#include "simulator/ros_io/measurement_publisher.hpp"

MeasurementPublisher::MeasurementPublisher(
  ros::NodeHandle& node, int agent_ID, int cam_ID)
    : _agent_ID(agent_ID), _cam_ID(cam_ID) {
  //   std::string wrench_topic_name = "wrench_" + std::to_string(agent_ID);
  //   _wrench_pub =
  //     node.advertise<geometry_msgs::Wrench>(wrench_topic_name, 32);
  //
  std::string view_topic_name = "view_" + std::to_string(agent_ID);
  _view_pub = node.advertise<sensor_msgs::Image>(view_topic_name, 32);

  std::string topic_name = "measurement_" + std::to_string(agent_ID);
  _measurement_pub =
    node.advertise<cv_project::robotMeasurement>(topic_name, 32);
}

void MeasurementPublisher::pub_view() {
  _view_pub.publish(_last_view_msg);
}

void MeasurementPublisher::pub_measurement(){
  _measurement_pub.publish(_last_measurement_msg);
}

void MeasurementPublisher::update_measurement(const robot_state_t& state, const mujoco_robot_wrench_t& robot_wrench) {
  auto time = ros::Time(state.t);

  _last_measurement_msg.header.stamp = time;
  _last_measurement_msg.header.frame_id = "robot_" + std::to_string(_agent_ID);

  _last_measurement_msg.end_pos = eigen_to_point_msg(state.end_pos);
  auto end_quat = Quaterniond(state.end_rot);
  end_quat.normalize();
  _last_measurement_msg.end_quat = eigen_to_quat_msg(end_quat);

  _last_measurement_msg.cam_pos = eigen_to_point_msg(state.cam_pos);
  auto cam_quat = Quaterniond(state.cam_rot);
  cam_quat.normalize();
  _last_measurement_msg.cam_quat = eigen_to_quat_msg(cam_quat);

  _last_measurement_msg.wrench = eigen_to_wrench_msg(robot_wrench);
}

void MeasurementPublisher::update_view(const cv::Mat& bgr_img, double time) {
  auto cam_ID = _agent_ID;
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_img).toImageMsg();
  msg->header.stamp = ros::Time(time);
  msg->header.frame_id = "camera_frame_" + std::to_string(cam_ID);

  _last_view_msg = msg;
}

std::shared_ptr<MeasurementPublisher> make_measurement_publisher(
  ros::NodeHandle node, int agent_ID, int cam_ID) {
  return std::make_shared<MeasurementPublisher>(node, agent_ID, cam_ID);
}