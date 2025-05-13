#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// #include "mujoco.h"  // mujoco-3.2.0
#include "mujoco/mujoco.h"  //mujoco-3.3.1
#include "utils/config.hpp"
#include "utils/math.hpp"

using namespace Eigen;

struct robot_FK_param_t {
  int DOF;

  // link screw & relpos
  std::vector<Vector3d> w;
  std::vector<Vector3d> p;
  //
  //   Vector3d w_0;
  //   Vector3d w_1;
  //   Vector3d w_2;
  //   Vector3d w_3;
  //   Vector3d w_4;
  //   Vector3d w_5;
  //
  //   Vector3d p_01;
  //   Vector3d p_12;
  //   Vector3d p_23;
  //   Vector3d p_34;
  //   Vector3d p_45;
  //   Vector3d p_5E;

  Vector3d base_pos;  // p_ss'
  Matrix3d base_rot;  // R_ss'

  void set_FK_param(
    const Vector3d& p_ss, const Matrix3d& R_ss,
    std::shared_ptr<config_t> config);
};

struct robot_state_t {
  double joint_pos_0;
  double joint_pos_1;
  double joint_pos_2;
  double joint_pos_3;
  double joint_pos_4;
  double joint_pos_5;

  void update_state(const mjData* d, int first_qpos_ID);
};

struct setpoint_t {
  Vector3d end_pos;
  Matrix3d end_rot;

  Vector3d end_vel;
  Vector3d end_w;
};

struct fk_result_t {
  setpoint_t endT;
  Eigen::MatrixXd Jes;  // Jacobian ([Jew; Jsv])
};


struct mujoco_robot_data_t {
  // TODO: define robot measurement data struct (image and wrench)
};

struct mujoco_control_t {
  double joint_pos_0;
  double joint_pos_1;
  double joint_pos_2;
  double joint_pos_3;
  double joint_pos_4;
  double joint_pos_5;

  void reset_control();
  mujoco_control_t operator+(const VectorXd& other) const;
  void clip_control(double u_bound, double l_bound);
};

struct mujoco_control_id_t {
  int _agent_ID;
  int _first_body_ID;
  int _first_act_ID;
  int _first_joint_ID;
  int _first_qpos_ID;
  int _first_DOF_ID;
  int _grasp_equality_ID;
  int _grasp_site_ID;

  void set_id(const mjModel* m, int agent_ID);
};
//
// mujoco_control_t inverse_kinematics(
//   const setpoint_t& setpoint, const setpoint_t& zero_config,
//   const robot_FK_param_t& param);
// setpoint_t forward_kinematics(
//   const mujoco_control_t& theta, const setpoint_t& zero_config,
//   const robot_FK_param_t& param);
// void set_control(mjData* d, const mujoco_control_t& control, int
// first_act_ID);

geometry_msgs::Point eigen_to_point_msg(Vector3d p);
geometry_msgs::Quaternion eigen_to_quat_msg(Quaterniond q);
geometry_msgs::Vector3 eigen_to_vector3_msg(Vector3d v);

Vector3d point_msg_to_eigen(geometry_msgs::Point point);
Quaterniond quat_msg_to_eigen(geometry_msgs::Quaternion quat);
Vector3d vector3_msg_to_eigen(geometry_msgs::Vector3 vec);