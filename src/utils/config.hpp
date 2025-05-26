#pragma once

#include <string>
#include <memory>

struct sim_param_t{
  double g;
  double FPS;
  bool measure_on;
  double measure_start_time;
  bool render_view;
  bool duration_check;
  bool data_gen_mode;
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
  robot_param_t robot;
  robot_IK_param_t IK;
  camera_param_t camera;
};

std::shared_ptr<config_t> get_config();
