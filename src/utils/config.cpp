#include "utils/config.hpp"

#include <yaml-cpp/yaml.h>
#include <ros/node_handle.h>

template <typename value_t>
inline value_t get_or_die(ros::NodeHandle& node, std::string name) {
  value_t value;
  if (!node.getParam(name, value)) {
    ROS_ERROR_STREAM(name << "undefined");
    throw std::out_of_range(name + "undefined");
  }
  return value;
}

sim_param_t get_sim_param(YAML::Node node) {
  return sim_param_t {
    .g = node["g"].as<double>(), .measure_on = node["measure_on"].as<bool>()};
}

robot_param_t get_robot_param(YAML::Node node) {
  return robot_param_t {
    .l01 = node["l01"].as<double>(),
    .l12x = node["l12x"].as<double>(),
    .l12y = node["l12y"].as<double>(),
    .l23 = node["l23"].as<double>(),
    .l34 = node["l34"].as<double>(),
    .l45 = node["l45"].as<double>(),
    .l5E = node["l5E"].as<double>()};
}

std::shared_ptr<config_t> get_config() {
  ros::NodeHandle node("~");

  const std::string param = get_or_die<std::string>(node, "param");
  const YAML::Node ynode = YAML::LoadFile(param);

  return std::make_shared<config_t>(config_t {
    .xml_file = get_or_die<std::string>(node, "xml_file"),
    .sim = get_sim_param(ynode["sim"]),
    .robot = get_robot_param(ynode["robot"])});
}