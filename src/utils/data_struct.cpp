#include "data_struct.hpp"

void robot_FK_param_t::set_FK_param(
  const Vector3d& p_ss, const Matrix3d& R_ss,
  std::shared_ptr<config_t> config) {
  DOF = 6;

  w.reserve(DOF);
  p.reserve(DOF);

  const auto e1 = Vector3d(1, 0, 0);
  const auto e2 = Vector3d(0, 1, 0);
  const auto e3 = Vector3d(0, 0, 1);

  w[0] = e3;
  w[1] = e2;
  w[2] = e2;
  w[3] = e2;
  w[4] = -e3;
  w[5] = e2;

  p[0] = config->robot.l01 * e3;  // p_01
  p[1] = Vector3d(config->robot.l12x, config->robot.l12y, 0);  // p_12
  p[2] = config->robot.l23 * e1;  // p_23
  p[3] = config->robot.l34 * e2;  // p_34
  p[4] = -config->robot.l45 * e3;  // p_45
  p[5] = config->robot.l5E * e2;  // p_5E

  base_pos = p_ss;
  base_rot = R_ss;
}

void robot_state_t::update_state(
  const mjData* d, int first_qpos_ID, int end_site_ID) {
  t = d->time;
  joint_pos_0 = d->qpos[first_qpos_ID];
  joint_pos_1 = d->qpos[first_qpos_ID + 1];
  joint_pos_2 = d->qpos[first_qpos_ID + 2];
  joint_pos_3 = d->qpos[first_qpos_ID + 3];
  joint_pos_4 = d->qpos[first_qpos_ID + 4];
  joint_pos_5 = d->qpos[first_qpos_ID + 5];

  end_pos << d->site_xpos[3 * end_site_ID], d->site_xpos[3 * end_site_ID + 1],
    d->site_xpos[3 * end_site_ID + 2];
  end_rot << d->site_xmat[9 * end_site_ID], d->site_xmat[9 * end_site_ID + 1],
    d->site_xmat[9 * end_site_ID + 2], d->site_xmat[9 * end_site_ID + 3],
    d->site_xmat[9 * end_site_ID + 4], d->site_xmat[9 * end_site_ID + 5],
    d->site_xmat[9 * end_site_ID + 6], d->site_xmat[9 * end_site_ID + 7],
    d->site_xmat[9 * end_site_ID + 8];
}

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

  std::string site_name = "attachment_site_" + std::to_string(_agent_ID);
  _grasp_site_ID = mj_name2id(m, mjOBJ_SITE, site_name.c_str());
  if (_grasp_site_ID == -1) {
    throw std::out_of_range("grasp site not found");
  }
}

mujoco_control_t mujoco_control_t::operator+(const VectorXd& other) const {
  if (other.size() != 6) {
    throw std::out_of_range(
      "VectorXd size mismatch for mujoco_control_t::operator+s");
  }

  return {joint_pos_0 + other(0), joint_pos_1 + other(1),
          joint_pos_2 + other(2), joint_pos_3 + other(3),
          joint_pos_4 + other(4), joint_pos_5 + other(5)};
}

void mujoco_control_t::clip_control(double u_bound, double l_bound) {
  joint_pos_0 = clip(joint_pos_0, u_bound, l_bound);
  joint_pos_1 = clip(joint_pos_1, u_bound, l_bound);
  joint_pos_2 = clip(joint_pos_2, u_bound, l_bound);
  joint_pos_3 = clip(joint_pos_3, u_bound, l_bound);
  joint_pos_4 = clip(joint_pos_4, u_bound, l_bound);
  joint_pos_5 = clip(joint_pos_5, u_bound, l_bound);
}

void mujoco_robot_wrench_t::init_wrench() {
  ext_force.setZero();
  ext_torque.setZero();
}

void mujoco_robot_wrench_t::update_wrench(const Vector3d& force, const Vector3d& torque){
  ext_force = force;
  ext_torque = torque;
}

// mujoco_control_t inverse_kinematics(
//   const setpoint_t& setpoint, const setpoint_t& zero_config,
//   const robot_FK_param_t& param) {
//   // TODO: implement inverse kinematics algorithm
// }
//
// setpoint_t forward_kinematics(const mujoco_control_t& theta, const
// setpoint_t& zero_config, const robot_FK_param_t& param){
//
// }
//
//
// void set_control(mjData* d, const mujoco_control_t& control, int
// first_act_ID) {
//   d->ctrl[first_act_ID] = control.joint_pos_0;
//   d->ctrl[first_act_ID + 1] = control.joint_pos_1;
//   d->ctrl[first_act_ID + 2] = control.joint_pos_2;
//   d->ctrl[first_act_ID + 3] = control.joint_pos_3;
//   d->ctrl[first_act_ID + 4] = control.joint_pos_4;
//   d->ctrl[first_act_ID + 5] = control.joint_pos_5;
// }

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