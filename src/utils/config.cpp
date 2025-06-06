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
    .g = node["g"].as<double>(),
    .sim_render_rate = node["sim_render_rate"].as<double>(),
    .duration_check = node["duration_check"].as<bool>(),
    .data_gen_mode = node["data_gen_mode"].as<bool>(),
    .separate_corr_mesh = node["separate_corr_mesh"].as<bool>()};
}

view_param_t get_view_param(YAML::Node node) {
  return view_param_t {
    .render_view = node["render_view"].as<bool>(),
    .view_pub_rate = node["view_pub_rate"].as<double>()};
}

measure_param_t get_measure_param(YAML::Node node) {
  return measure_param_t {
    .measure_on = node["measure_on"].as<bool>(),
    .measure_pub_rate = node["measure_pub_rate"].as<double>(),
    .measure_start_time = node["measure_start_time"].as<double>()};
}

mesh_sim_param_t get_mesh_param(YAML::Node node) {
  return mesh_sim_param_t {
    .GT_mesh = node["GT_mesh"].as<bool>(),
    .mesh_init_time = node["mesh_init_time"].as<double>(),
    .spacing = node["spacing"].as<double>(),
    .rows = node["rows"].as<int>(),
    .cols = node["cols"].as<int>(),
    .m_0 = node["m_0"].as<double>(),
    .k_0 = node["k_0"].as<double>(),
    .beta_0 = node["beta_0"].as<double>()};
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

robot_IK_param_t get_IK_param(YAML::Node node) {
  return robot_IK_param_t {
    .lambda = node["lambda"].as<double>(),
    .Kw = node["Kw"].as<double>(),
    .Kp = node["Kp"].as<double>()};
}

camera_param_t get_camera_param(YAML::Node node) {
  auto param = camera_param_t {
    .fovy = node["fovy"].as<int>(),
    .width = node["width"].as<int>(),
    .height = node["height"].as<int>()};

  param.focal_pixel =
    param.height / (2 * std::tan(param.fovy / 2 * 3.141592 / 180));
  param.K << -param.focal_pixel, 0, param.width / 2, 0, param.focal_pixel,
    param.height / 2, 0, 0, 1;

  return param;
}
Eigen::MatrixXd camera_param_t::compute_P(
  const Eigen::Vector3d& cam_pos, const Eigen::Matrix3d& cam_rot) const {
  Eigen::Matrix4d T_cs;
  T_cs.setIdentity();
  T_cs.topLeftCorner(3, 3) = cam_rot;
  T_cs.topRightCorner(3, 1) = -cam_rot * cam_pos;
  Eigen::Matrix<double, 3, 4> I_0;
  I_0.topLeftCorner(3, 3).setIdentity();
  I_0.topRightCorner(3, 1).setZero();
  return K * I_0 * T_cs;
}

XPBD_param_t get_xpbd_param(YAML::Node node) {
  return XPBD_param_t {
    .max_iter = node["max_iter"].as<int>(),
    .correction_iter = node["correction_iter"].as<int>(),
    .visual_correction_on = node["visual_correction_on"].as<bool>(),
    .conv_crit = node["conv_crit"].as<double>(),
    .alpha_chamfer = node["alpha_chamfer"].as<double>(),
    .beta_chamfer = node["beta_chamfer"].as<double>()};
}

std::shared_ptr<config_t> get_config() {
  ros::NodeHandle node("~");

  const std::string param = get_or_die<std::string>(node, "param");
  const YAML::Node ynode = YAML::LoadFile(param);

  return std::make_shared<config_t>(config_t {
    .xml_file = get_or_die<std::string>(node, "xml_file"),
    .sim = get_sim_param(ynode["sim"]),
    .view = get_view_param(ynode["view"]),
    .measure = get_measure_param(ynode["measure"]),
    .mesh = get_mesh_param(ynode["mesh"]),
    .robot = get_robot_param(ynode["robot"]),
    .IK = get_IK_param(ynode["IK"]),
    .camera = get_camera_param(ynode["camera"]),
    .xpbd = get_xpbd_param(ynode["XPBD"])});
}