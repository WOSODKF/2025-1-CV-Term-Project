#pragma once

#include <string>
#include <memory>

struct sim_param_t{
  double g;
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

// namespace inrol {
//   struct sim_param_t {
//     double m;
//     double g;
//   };
// 
//   struct position_control_param_t {
//     double k_p;
//     double k_v;
//     double k_p_z;
//     double k_v_z;
// 
//     double beta;
//     double epsilon;
// 
//     double alpha;
//     double gamma;
//   };
// 
//   struct tool_orientation_control_param_t {
//     double k_Rt;
//   };
// 
//   struct body_rate_control_param_t {
//     double k_w;
//     double k_I_w;
//   };
// 
//   struct thrust_mixer_param_t {
//     double r;
//     double rho;
//   };
// 
//   struct config_t {
//     std::string xml_file;
// 
//     sim_param_t sim;
//     position_control_param_t position;
//     tool_orientation_control_param_t tool_orientation;
//     body_rate_control_param_t body_rate;
//     thrust_mixer_param_t thrust_mixer;
//   };
// 
//   std::shared_ptr<config_t> get_config();
// }  // namespace inrol
