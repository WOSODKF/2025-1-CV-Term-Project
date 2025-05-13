#include "simulator/robot.hpp"

Robot::Robot(
  const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
  ros::NodeHandle& node, int agent_ID)
    : _agent_ID(agent_ID) {
  _last_control.reset_control();
  _mj_ID.set_id(m, agent_ID);

  int body_ID = _mj_ID._first_body_ID;
  auto base_pos = Eigen::Vector3d(
    d->xpos[3 * body_ID], d->xpos[3 * body_ID + 1], d->xpos[3 * body_ID + 2]);
  auto base_quat = Eigen::Quaterniond(
    d->xquat[4 * body_ID], d->xquat[4 * body_ID + 1], d->xquat[4 * body_ID + 2],
    d->xquat[4 * body_ID + 3]);
  base_quat.normalize();
  auto base_rot = base_quat.toRotationMatrix();

  _fk_param.set_FK_param(base_pos, base_rot, config);
  _setpoint_sub = make_setpoint_subscriber(node, agent_ID);
}

void Robot::set_mujoco_control(const mjModel* m, mjData* d) {
  compute_control(m, d);
  // std::cout << "current attachment position:"
  //           << d->site_xpos[3 * _mj_ID._grasp_site_ID] << " "
  //           << d->site_xpos[3 * _mj_ID._grasp_site_ID + 1] << " "
  //           << d->site_xpos[3 * _mj_ID._grasp_site_ID + 2] << std::endl;
  // std::cout << "base pos:" << d->xpos[3 * _mj_ID._first_body_ID] << " "
  //           << d->xpos[3 * _mj_ID._first_body_ID + 1] << " "
  //           << d->xpos[3 * _mj_ID._first_body_ID + 2] << std::endl;

  int first_act_ID = _mj_ID._first_act_ID;

  d->ctrl[first_act_ID] = _last_control.joint_pos_0;
  d->ctrl[first_act_ID + 1] = _last_control.joint_pos_1;
  d->ctrl[first_act_ID + 2] = _last_control.joint_pos_2;
  d->ctrl[first_act_ID + 3] = _last_control.joint_pos_3;
  d->ctrl[first_act_ID + 4] = _last_control.joint_pos_4;
  d->ctrl[first_act_ID + 5] = _last_control.joint_pos_5;
}

const mujoco_control_t Robot::inverse_kinematics(
  const setpoint_t& setpoint, double dt) {
  // IK hyperparams
  const double lambda = 2.0;
  const double Kw = 1.0;
  const double Kp = 3.0;

  // desired config
  auto Rd = setpoint.end_rot;
  auto pd = setpoint.end_pos;

  // initial value
  mujoco_control_t control_guess {
    .joint_pos_0 = _current_state.joint_pos_0,
    .joint_pos_1 = _current_state.joint_pos_1,
    .joint_pos_2 = _current_state.joint_pos_2,
    .joint_pos_3 = _current_state.joint_pos_3,
    .joint_pos_4 = _current_state.joint_pos_4,
    .joint_pos_5 = _current_state.joint_pos_5};

  // solve forward kinematics
  auto fk_result = forward_kinematics(control_guess);

  auto Jes = fk_result.Jes;
  auto Rse = fk_result.endT.end_rot;
  auto pse = fk_result.endT.end_pos;

  // compute error
  auto e_w = matLogSO3(Rse.transpose() * Rd);
  auto e_p = pd - pse;
  VectorXd e(6);
  e << Kw * e_w, Kp * e_p;

  // update by single step Levenberg-Marquardt(damped least square)
  auto dTheta =
    (Jes.transpose() * Jes +
     lambda * Eigen::Matrix<double, 6, 6>::Identity())  // 6 = DOF(hardcoding)
      .inverse() *
    Jes.transpose() * e;
  control_guess = control_guess + dTheta;

  return control_guess;
}

const fk_result_t Robot::forward_kinematics(const mujoco_control_t& theta) {
  // only position kinematics
  std::vector<double> _theta = {theta.joint_pos_0, theta.joint_pos_1,
                                theta.joint_pos_2, theta.joint_pos_3,
                                theta.joint_pos_4, theta.joint_pos_5};

  Eigen::Matrix4d Tss, Ti0, Ti, Tsi, Tse, skewA;  // SE(3) transforms
  Eigen::VectorXd Ai;  // link screw
  Eigen::MatrixXd Js, Je, Jes;  // full space/end-effector Jacobian/[Jew; Jsv]

  Ai.resize(6);
  Js.resize(6, _fk_param.DOF);

  Tss.setIdentity();
  Tss.topLeftCorner(3, 3) = _fk_param.base_rot;
  Tss.topRightCorner(3, 1) = _fk_param.base_pos;

  // global to local base
  Tsi = Tss;

  // btw adjacent links
  for (int i = 0; i < _fk_param.DOF; i++) {
    Ti0.setIdentity();
    if (i > 0) {
      Ti0.topRightCorner(3, 1) = _fk_param.p[i - 1];
    }

    Ai.setZero();
    Ai.topRightCorner(3, 1) = _fk_param.w[i];
    skewA = skew6(Ai);

    Ti = Ti0 * (_theta[i] * skewA).exp();
    Tsi = Tsi * Ti;

    // fill Jacobian
    Js.block<6, 1>(0, i) = AdSE3(Tsi) * Ai;
  }

  // end-effector
  Ti.setIdentity();
  // Ti.topLeftCorner(3, 3) << -1, 0, 0, 0, 0, 1, 0, 1, 0; // deprecated
  Ti.topRightCorner(3, 1) = _fk_param.p[5];

  Tse = Tsi * Ti;

  Je = AdSE3(invSE3(Tse)) * Js;  // convert to end-effector Jacobian

  // convert to mixed Jacobian
  Eigen::Matrix<double, 6, 6> conv;
  conv.topLeftCorner(3, 3).setIdentity();
  conv.bottomRightCorner(3, 3) = Tse.topLeftCorner(3, 3);
  Jes = conv * Je;

  setpoint_t endT;
  endT.end_pos = Tse.topRightCorner(3, 1);
  endT.end_rot = Tse.topLeftCorner(3, 3);
  endT.end_vel.setZero();
  endT.end_w.setZero();

  fk_result_t result;
  result.endT = endT;
  result.Jes = Jes;

  return result;
}

void Robot::compute_control(const mjModel* m, const mjData* d) {
  // update state
  _current_state.update_state(d, _mj_ID._first_qpos_ID);

  // compute control
  auto dt = m->opt.timestep;
  _last_control = inverse_kinematics(_setpoint_sub->setpoint(), dt);
}

const mujoco_control_t Robot::get_control() const {
  return _last_control;
}

const mujoco_control_id_t Robot::get_id() const {
  return _mj_ID;
}

std::shared_ptr<Robot> make_robot(
  const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
  ros::NodeHandle& node, int agent_ID) {
  return std::make_shared<Robot>(m, d, config, node, agent_ID);
}