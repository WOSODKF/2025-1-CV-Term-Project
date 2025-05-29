#pragma once

#include <string>
#include <memory>

struct sim_param_t{
  double g;
  double sim_render_rate;
  bool duration_check;
  bool data_gen_mode;
};

struct view_param_t {
  bool render_view;
  double view_pub_rate;
};

struct measure_param_t{
  bool measure_on;
  double measure_pub_rate;
  double measure_start_time;
};

struct mesh_sim_param_t {
  bool GT_mesh;
  double mesh_init_time;
  int rows;
  int cols;
};

struct robot_param_t {
  double l01;
  double l12x;
  double l12y;
  double l23;
  double l34;
  double l45;
  double l5E;
};

struct robot_IK_param_t{
  double lambda;
  double Kw;
  double Kp;
};

struct camera_param_t{
  int fovy;
  int width;
  int height;
};

struct config_t {
  std::string xml_file;
  sim_param_t sim;
  view_param_t view;
  measure_param_t measure;
  mesh_sim_param_t mesh;
  robot_param_t robot;
  robot_IK_param_t IK;
  camera_param_t camera;
};

std::shared_ptr<config_t> get_config();
