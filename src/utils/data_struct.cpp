#include "data_struct.hpp"

void mujoco_control_t::reset_control() {
  joint_pos_0 = 0.0;
  joint_pos_1 = 0.0;
  joint_pos_2 = 0.0;
  joint_pos_3 = 0.0;
  joint_pos_4 = 0.0;
  joint_pos_5 = 0.0;
}

void mujoco_control_id_t::set_id(const mjModel* m, int agent_ID) {
  _agent_ID = agent_ID;

  std::string first_body_name = "base_" + std::to_string(_agent_ID);
  _first_body_ID = mj_name2id(m, mjOBJ_BODY, first_body_name.c_str());
  if (_first_body_ID == -1) {
    throw std::out_of_range("body not found");
  }

  std::string first_act_name = "shoulder_pan_" + std::to_string(_agent_ID);
  _first_act_ID = mj_name2id(m, mjOBJ_ACTUATOR, first_act_name.c_str());
  if (_first_act_ID == -1) {
    throw std::out_of_range("actuator not found");
  }

  std::string first_joint_name =
    "shoulder_pan_joint_" + std::to_string(_agent_ID);
  _first_joint_ID = mj_name2id(m, mjOBJ_JOINT, first_joint_name.c_str());
  if (_first_joint_ID == -1) {
    throw std::out_of_range("joint not found");
  }
  _first_qpos_ID = m->jnt_qposadr[_first_joint_ID];
  _first_DOF_ID = m->jnt_dofadr[_first_joint_ID];

  std::string equality_name = "grasp_" + std::to_string(_agent_ID);
  _grasp_equality_ID = mj_name2id(m, mjOBJ_EQUALITY, equality_name.c_str());
  if (_grasp_equality_ID == -1) {
    throw std::out_of_range("grasp equality not found");
  }
}

mujoco_control_t inverse_kinematics(setpoint_t setpoint) {
  // TODO: implement inverse kinematics algorithm
}

void set_control(mjData* d, const mujoco_control_t& control, int first_act_ID) {
  d->ctrl[first_act_ID] = control.joint_pos_0;
  d->ctrl[first_act_ID + 1] = control.joint_pos_1;
  d->ctrl[first_act_ID + 2] = control.joint_pos_2;
  d->ctrl[first_act_ID + 3] = control.joint_pos_3;
  d->ctrl[first_act_ID + 4] = control.joint_pos_4;
  d->ctrl[first_act_ID + 5] = control.joint_pos_5;
}

geometry_msgs::Point eigen_to_point_msg(Vector3d p) {
  geometry_msgs::Point result;

  result.x = p.x();
  result.y = p.y();
  result.z = p.z();

  return result;
}

geometry_msgs::Quaternion eigen_to_quat_msg(Quaterniond q) {
  geometry_msgs::Quaternion result;

  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();

  return result;
}

geometry_msgs::Vector3 eigen_to_vector3_msg(Vector3d v) {
  geometry_msgs::Vector3 result;

  result.x = v.x();
  result.y = v.y();
  result.z = v.z();

  return result;
}

Vector3d point_msg_to_eigen(geometry_msgs::Point point) {
  return Vector3d(point.x, point.y, point.z);
}

Quaterniond quat_msg_to_eigen(geometry_msgs::Quaternion quat) {
  auto result = Quaterniond(quat.w, quat.x, quat.y, quat.z);
  result.normalize();

  return result;
}

Vector3d vector3_msg_to_eigen(geometry_msgs::Vector3 vec) {
  return Vector3d(vec.x, vec.y, vec.z);
}