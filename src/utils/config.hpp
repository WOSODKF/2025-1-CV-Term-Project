#pragma once

#include <string>
#include <memory>

struct sim_param_t{
  double g;
  bool measure_on;
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

struct config_t{
  std::string xml_file;
  sim_param_t sim;
  robot_param_t robot;
};

std::shared_ptr<config_t> get_config();
