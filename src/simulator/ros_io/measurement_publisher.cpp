#include "simulator/ros_io/measurement_publisher.hpp"

MeasurementPublisher::MeasurementPublisher(ros::NodeHandle& node, int agent_ID){
  std::string wrench_topic_name = "wrench_" + std::to_string(agent_ID);
  _wrench_pub =
    node.advertise<cv_project::robotWrench>(wrench_topic_name, 32);

  std::string view_topic_name = "view_" + std::to_string(agent_ID);
  _view_pub = node.advertise<sensor_msgs::Image>(view_topic_name, 32);
}

void MeasurementPublisher::pub() {
  _wrench_pub.publish(_last_wrench_msg);
  _view_pub.publish(_last_view_msg);
}

void MeasurementPublisher::update_wrench(const mujoco_robot_wrench_t& robot_wrench) {
  _last_wrench_msg.ext_force =
    eigen_to_vector3_msg(robot_wrench.ext_force);
  _last_wrench_msg.ext_torque =
    eigen_to_vector3_msg(robot_wrench.ext_torque);
}

void MeasurementPublisher::update_view(const cv::Mat& rgb_img){
  cv::Mat rgb_flipped, bgr_img;
  cv::flip(rgb_img, rgb_flipped, 0);
  cv::cvtColor(rgb_flipped, bgr_img, cv::COLOR_RGB2BGR);

  auto cam_ID = _agent_ID;
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_img).toImageMsg();
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "camera_frame_" + std::to_string(cam_ID);

  _last_view_msg = msg;
}

std::shared_ptr<MeasurementPublisher> make_measurement_publisher(
  ros::NodeHandle node, int agent_ID) {
  return std::make_shared<MeasurementPublisher>(node, agent_ID);
}