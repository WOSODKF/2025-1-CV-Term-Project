#include "simulator/robot.hpp"

Robot::Robot(
  const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
  ros::NodeHandle& node, int agent_ID)
    : _agent_ID(agent_ID) {
  _last_control.reset_control();
  _mj_ID.set_id(m, agent_ID);
  _render_view = config->sim.render_view;

  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm* tm = std::localtime(&now_c);

  std::stringstream ss;
  ss << std::put_time(tm, "%m%d_%H%M");
  _dataset_code = ss.str();

  int body_ID = _mj_ID._first_body_ID;
  auto base_pos = Eigen::Vector3d(
    d->xpos[3 * body_ID], d->xpos[3 * body_ID + 1], d->xpos[3 * body_ID + 2]);
  auto base_quat = Eigen::Quaterniond(
    d->xquat[4 * body_ID], d->xquat[4 * body_ID + 1], d->xquat[4 * body_ID + 2],
    d->xquat[4 * body_ID + 3]);
  base_quat.normalize();
  auto base_rot = base_quat.toRotationMatrix();

  _ik_param = config->IK;
  _fk_param.set_FK_param(base_pos, base_rot, config);
  _setpoint_sub = make_setpoint_subscriber(node, agent_ID);
  _measurement_pub = make_measurement_publisher(node, agent_ID, _mj_ID._cam_ID);
  _state_pub = make_state_publisher(node, agent_ID);

  _last_wrench.init_wrench();
}

void Robot::compute_control(const mjModel* m, const mjData* d) {
  // update state
  _current_state.update_state(
    d, _mj_ID._first_qpos_ID, _mj_ID._grasp_site_ID, _mj_ID._cam_ID);
  // measurement (may need some computation / MBO / etc.)
  // _measured_data.update_data(force, torque);
  // _measured_wrench.init_wrench();  // temp

  // compute control
  auto dt = m->opt.timestep;
  _last_control = inverse_kinematics(_setpoint_sub->setpoint(), dt);
}

// void Robot::update_robot_data(const mjModel* m, const mjData* d) {
//   // TODO: update state & measurement
//
//   // state
//   _current_state.update_state(d, _mj_ID._first_qpos_ID,
//   _mj_ID._grasp_site_ID);
//
//
// }

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

void Robot::update_wrench(const mjModel* m, mjData* d) {
  Eigen::Vector3d force, torque;
  //(temp)
  // force.setZero();
  // torque.setZero();
  /*----TODO: get constraint force and torque from efc_force-----*/
  // if (_agent_ID == 1) {
  //   std::string name = "grasp_" + std::to_string(_agent_ID);
  //   int equality_id = mj_name2id(m, mjOBJ_EQUALITY, name.c_str());
  //   std::cout << "nefc:" << d->nefc << std::endl;
  //   std::cout << "equality ID:" << equality_id << std::endl;
  //   std::cout << "grasp_equality_ID: " << _mj_ID._grasp_equality_ID
  //             << std::endl;
  //   std::cout << "efc_force:" << std::endl;
  //   // for (int i = 0; i < 6; i++) {
  //   //   std::cout << d->efc_force[6*equality_id + i] << " ";
  //   // }
  //   for (int i = 0; i < 24; i++) {
  //     std::cout << d->efc_force[i] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  /* Note
  - efc_force includes constraints in cloth
  - but equality ID for grasp does not change
  - may have to implement MBO...
  */
  auto equality_ID = _mj_ID._grasp_equality_ID;
  force << d->efc_force[6 * equality_ID], d->efc_force[6 * equality_ID + 1],
    d->efc_force[6 * equality_ID + 2];
  torque << d->efc_force[6 * equality_ID + 3],
    d->efc_force[6 * equality_ID + 4], d->efc_force[6 * equality_ID + 5];

  /*-------------------------------------------------------------*/
  _last_wrench.update_wrench(force, torque);
}

void Robot::update_view(
  const mjModel* m, mjData* d, mjvOption& opt, mjvCamera& cam, mjvScene& scn,
  mjrContext& con, const mjrRect& viewport, bool data_gen_mode) {
  int cam_ID = _agent_ID;
  const int WIDTH = viewport.width;
  const int HEIGHT = viewport.height;
  unsigned char rgb_buffer[WIDTH * HEIGHT * 3];

  if (!_render_view) {
    _last_view = cv::Mat(HEIGHT, WIDTH, CV_8UC3, rgb_buffer);
    return;
  }

  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = cam_ID;

  // Update scene
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

  // Render
  mjr_render(viewport, &scn, &con);

  // Read pixels (RGB only here)
  mjr_readPixels(rgb_buffer, NULL, viewport, &con);

  // Get image
  cv::Mat rgb_img(HEIGHT, WIDTH, CV_8UC3, rgb_buffer);
  cv::Mat rgb_flipped, bgr_img;
  cv::flip(rgb_img, rgb_flipped, 0);
  cv::cvtColor(rgb_flipped, bgr_img, cv::COLOR_RGB2BGR);

  _last_view = bgr_img;

  // storing view as image
  if (data_gen_mode) {
    std::string dirname = ros::package::getPath("cv_project");
    dirname = dirname + "/training_data/25" + _dataset_code;

    if (!std::filesystem::exists(dirname)) {
      if (!std::filesystem::create_directory(dirname)) {
        std::cerr << "Failed to create directory!" << std::endl;
      }
    }

    dirname = dirname + "/images/";

    if (!std::filesystem::exists(dirname)) {
      if (std::filesystem::create_directory(dirname)) {
        std::cout << "Directory created: " << dirname << std::endl;
      } else {
        std::cerr << "Failed to create directory!" << std::endl;
      }
    }

    std::string filename = _dataset_code + "_view_" +
      std::to_string(_agent_ID) + "_" + std::to_string(d->time) + ".png";
    bool success = cv::imwrite(dirname + filename, bgr_img);
    if (success) {
      std::cout << "Image stored in " << dirname + filename << std::endl;
    } else {
      std::cout << "Failed storing image at " << dirname + filename
                << std::endl;
    }
  }
}

void Robot::publish_state() {
  _state_pub->update_state(_current_state);
  _state_pub->pub();
}

void Robot::publish_measurement() {
  _measurement_pub->update(_current_state, _last_wrench, _last_view);
  _measurement_pub->pub();
}

/* Kinematics */
const mujoco_control_t Robot::inverse_kinematics(
  const setpoint_t& setpoint, double dt) {
  // IK hyperparams
  const double lambda = _ik_param.lambda;
  const double Kw = _ik_param.Kw;
  const double Kp = _ik_param.Kp;

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