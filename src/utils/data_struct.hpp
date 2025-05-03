#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// #include "mujoco.h"  // mujoco-3.2.0
#include "mujoco/mujoco.h"  //mujoco-3.3.1

using namespace Eigen;

struct setpoint_t{
  Vector3d end_pos;
  Vector3d end_vel;
};

struct mujoco_robot_data_t{
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
};

struct mujoco_control_id_t{
  int _agent_ID;
  int _first_body_ID;
  int _first_act_ID;
  int _first_joint_ID;
  int _first_qpos_ID;
  int _first_DOF_ID;
  int _grasp_equality_ID;

  void set_id(const mjModel* m, int agent_ID);
};

mujoco_control_t inverse_kinematics(setpoint_t setpoint);
void set_control(mjData* d, const mujoco_control_t& control, int first_act_ID);

geometry_msgs::Point eigen_to_point_msg(Vector3d p);
geometry_msgs::Quaternion eigen_to_quat_msg(Quaterniond q);
geometry_msgs::Vector3 eigen_to_vector3_msg(Vector3d v);

Vector3d point_msg_to_eigen(geometry_msgs::Point point);
Quaterniond quat_msg_to_eigen(geometry_msgs::Quaternion quat);
Vector3d vector3_msg_to_eigen(geometry_msgs::Vector3 vec);