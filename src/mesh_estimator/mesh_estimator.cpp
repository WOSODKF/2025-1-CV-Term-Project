#include "mesh_estimator/mesh_estimator.hpp"

MeshEstimator::MeshEstimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node) {
  if (node.getParam("agent_num", _agent_num)) {
  } else {
    ROS_ERROR("/mesh_estimator/agent_num is not found");
  }
  _measure_rate = config->measure.measure_pub_rate;
  _view_rate = config->view.view_pub_rate;
  _camera_param = config->camera;

  _mask = std::vector<mask_data_t>(_agent_num);
  _measurement = std::vector<robot_measurement_t>(_agent_num);
  _prev_measurement = std::vector<robot_measurement_t>(_agent_num);

  _mask_sub.resize(_agent_num);
  _measure_sub.resize(_agent_num);
  _mask_ready.resize(_agent_num);
  _measurement_ready.resize(_agent_num);

  for (int i = 0; i < _agent_num; i++) {
    _mask_sub[i] = make_mask_subscriber(node, config, i);
    _mask_sub[i]->set_flag_callback(
      std::bind(
        &MeshEstimator::mask_receiving_callback, this, std::placeholders::_1));
    _mask_ready[i] = false;

    _measure_sub[i] = make_measurement_subscriber(node, i);
    _measure_sub[i]->set_flag_callback(
      std::bind(
        &MeshEstimator::measure_receiving_callback, this,
        std::placeholders::_1));
    _measurement_ready[i] = false;
  }

  _initial_mesh_sub = make_initial_mesh_subscriber(node, config);

  _undeformed_mesh.set(config);
  _mesh_param.init(config);
  _vertex_num = _mesh_param.rows * _mesh_param.cols;
  _last_vels.resize(_vertex_num);
  _last_mesh.init(-1, _mesh_param.rows, _mesh_param.cols);
  for (int i = 0; i < _vertex_num; i++) {
    _last_vels[i].setZero();
  }

  _mesh_pub = make_mesh_publisher(node, "estimated", _last_mesh);

  _xpbd_param = config->xpbd;
  _xpbd_param.strain_constraint_num =
    3 * (_mesh_param.rows - 1) * (_mesh_param.cols - 1) +
    (_mesh_param.rows - 1) + (_mesh_param.cols - 1);

  // 4 * (_mesh_param.rows - 1) * (_mesh_param.cols - 1) +
  // (_mesh_param.rows - 1) + (_mesh_param.cols - 1); // for diagonal 2
  // 6 * (_mesh_param.rows - 1) * (_mesh_param.cols - 1); // for SVK energy

  _g = config->sim.g;

  _initial_mesh_ready = false;
  _all_mask_ready = false;

  /* TODO: assign callback to measurement subscriber? (event-based running &
   * only ros::spin() in main fn.)*/
}

// void MeshEstimator::run() {
//   while (ros::ok()) {
//     /* main loop */
//
//     /* TODO: delay until seg node and mesh initialized (receive init message
//      * from seg node) s*/
//     // update_data();
//     update_mesh();
//     // publish_data();
//   }
// }

/* Functions
- From received data(masks, wrenches and poses) ...
- Update mesh
- Publish data
*/

void MeshEstimator::update_mesh() {
  /* TODO: run PBD / if mask received - run correction */
  compute_XPBD();
  // if(mask_received()){
  //   correct_mesh();
  // }
}

// void MeshEstimator::update_data() {
//   // update data to recently received data
//   for (int i = 0; i < _agent_num; i++) {
//     _mask[i] = _mask_sub[i]->get_data();  // judge if mask is new
//     _measurement[i] = _measure_sub[i]->get_data();
//   }
//   // _mesh_param = _param_sub->get_data();
// }

void MeshEstimator::publish_data() {
  //   _err_pub->update();
  // _err_pub->pub();
}

void MeshEstimator::measure_receiving_callback(const std::string& frame_id) {
  // frame_id format: "robot_n"
  int id = std::stoi(frame_id.substr(frame_id.find('_') + 1));
  _measurement_ready[id] = true;
  _measurement[id] = _measure_sub[id]->get_data();
  bool all_ready = true;
  for (int i = 0; i < _agent_num; i++) {
    all_ready = all_ready && _measurement_ready[i];
  }

  // timestamp validation
  if (all_ready) {
    for (int i = 1; i < _agent_num; i++) {
      all_ready = all_ready &&
        _measure_sub[i - 1]->get_data().t == _measure_sub[i]->get_data().t;
    }
    if (!all_ready) {
      return;
    }
  }

  if (!_initial_mesh_ready) {
    _initial_mesh_ready = _initial_mesh_sub->initial_mesh_ready();
    if (_initial_mesh_ready) {
      _last_mesh = _initial_mesh_sub->get_data();
    } else {
      return;
    }
  }

  // 0602 checked: time sync is well matched
  if (all_ready) {
    // publish mesh of last timestep
    if (_all_mask_ready) {
      correct_mesh();
      _all_mask_ready = false;
      for (int i = 0; i < _agent_num; i++) {
        _mask_ready[i] = false;
      }
    }
    _mesh_pub->update(_last_mesh);
    _mesh_pub->pub();

    update_mesh();
    for (int i = 0; i < _agent_num; i++) {
      _measurement_ready[i] = false;
    }
  }
}

void MeshEstimator::mask_receiving_callback(const std::string& frame_id) {
  // frame_id format: "camera_frame_n"
  int id = std::stoi(frame_id.substr(frame_id.rfind('_') + 1));
  _mask_ready[id] = true;
  _mask[id] = _mask_sub[id]->get_data();
  bool all_ready = true;
  for (int i = 0; i < _agent_num; i++) {
    all_ready = all_ready && _mask_ready[i];
  }

  // timestamp validation
  if (all_ready) {
    for (int i = 1; i < _agent_num; i++) {
      all_ready = all_ready &&
        _mask_sub[i - 1]->get_data().t == _mask_sub[i]->get_data().t;
    }
  }

  _all_mask_ready = all_ready;
}

/* Mask correction */
void MeshEstimator::correct_mesh() {
  /* Apply mask projection constraint */
  const int rows = _mesh_param.rows;
  const int cols = _mesh_param.cols;
  const int cam_height = _camera_param.height;
  const int cam_width = _camera_param.width;
  auto points = _last_mesh.points;
  const auto prev_points = _last_mesh.points;

  MatrixXi projection;
  projection.resize(_camera_param.height, _camera_param.width);
  for (int i = 0; i < _agent_num; i++) {
    projection.setZero();
    auto cam_pos = _prev_measurement[i].cam_pos;
    auto cam_rot = _prev_measurement[i].cam_rot;
    auto cam_P = _camera_param.compute_P(cam_pos, cam_rot);
    for (int row = 0; row < rows; row++) {
      for (int col = 0; col < cols; col++) {
        Vector4d hom_pos;
        hom_pos << points[col + cols * row], 1;
        auto proj_pos = cam_P * hom_pos;
        if (proj_pos(2) > 0.01) {
          proj_pos /= proj_pos(2);
        } else {
          continue;
        }
        int proj_col = static_cast<int>(std::round(proj_pos(0)));
        int proj_row = static_cast<int>(std::round(proj_pos(1)));
        if (
          proj_col > cam_width || proj_col < 0 || proj_row > cam_height ||
          proj_row < 0) {
          continue;
        }
        projection(proj_row, proj_col) = 1;
      }
    }
    /* compute soft mIoU */
    
  }

  /* minimal XPBD with _prev_measurement */
}

/* XBPD computation */
void MeshEstimator::compute_XPBD() {
  const double m_i = _mesh_param.m / _vertex_num;
  const double dt = 1.0 / _measure_rate;
  const double k = _mesh_param.k;  // alpha = 1/k
  const double gamma = _mesh_param.beta / (_mesh_param.k * dt);
  const double l_0 = _mesh_param.spacing;
  const int rows = _mesh_param.rows;
  const int cols = _mesh_param.cols;

  auto points = _last_mesh.points;
  const auto prev_points = _last_mesh.points;
  std::vector<double> lambda(_xpbd_param.strain_constraint_num, 0.0);

  /* Give predictions by external force */
  for (int i = 0; i < _vertex_num; i++) {
    points[i] += dt * _last_vels[i] - dt * dt * _g * Vector3d::UnitZ() / 2;

    // grasp points - Force is not required, if position constraint is given
    // if (i == 0) {  // grasp 2
    //   points[i] += -dt * dt * _measurement[2].ext_force;
    // }
    // if (i == _mesh_param.rows - 1) {  // grasp 3
    //   points[i] += -dt * dt * _measurement[3].ext_force;
    // }
    // if (i == _mesh_param.rows * _mesh_param.cols - _mesh_param.cols) {  //
    // grasp
    //                                                                     // 1
    //   points[i] += -dt * dt * _measurement[1].ext_force;
    // }
    // if (i == _mesh_param.rows * _mesh_param.cols - 1) {  // grasp 0
    //   points[i] += -dt * dt * _measurement[0].ext_force;
    // }
  }

  /* Constraint projection */
  for (int i = 0; i < _xpbd_param.max_iter; i++) {
    /* energy constraint */
    int con_idx = 0;
    for (int row = 0; row < rows - 1; row++) {
      for (int col = 0; col < cols - 1; col++) {
        Vector3d& x0 = points[col + cols * row];
        Vector3d& x1 = points[col + 1 + cols * row];
        Vector3d& x2 = points[col + 1 + cols * (row + 1)];
        Vector3d& x3 = points[col + cols * (row + 1)];

        Vector3d x0_prev = prev_points[col + cols * row],
                 x1_prev = prev_points[col + 1 + cols * row],
                 x2_prev = prev_points[col + 1 + cols * (row + 1)],
                 x3_prev = prev_points[col + cols * (row + 1)];

        /* orthogonal 1 */
        compute_energy_constraint_projection(
          x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i / 2, gamma);
        con_idx++;

        /* diagonal */
        compute_energy_constraint_projection(
          x0, x2, x0_prev, x2_prev, lambda[con_idx], sqrt(2) * l_0, k, m_i / 2,
          gamma);
        con_idx++;

        /* orthogonal 2 */
        compute_energy_constraint_projection(
          x0, x3, x0_prev, x3_prev, lambda[con_idx], l_0, k, m_i / 2, gamma);
        con_idx++;

        // /* diagonal 2? */
        // compute_energy_constraint_projection(
        //   x1, x3, x1_prev, x3_prev, lambda[con_idx], sqrt(2) * l_0, k, m_i /
        //   2, gamma);
        // con_idx++;
      }
    }
    for (int row = 0; row < rows - 1; row++) {
      Vector3d& x0 = points[cols - 1 + cols * row];
      Vector3d& x1 = points[cols - 1 + cols * (row + 1)];
      Vector3d x0_prev = prev_points[cols - 1 + cols * row],
               x1_prev = prev_points[cols - 1 + cols * (row + 1)];
      compute_energy_constraint_projection(
        x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i / 2, gamma);
      con_idx++;
    }
    for (int col = 0; col < cols - 1; col++) {
      Vector3d& x0 = points[col + cols * (rows - 1)];
      Vector3d& x1 = points[col + 1 + cols * (rows - 1)];
      Vector3d x0_prev = prev_points[col + cols * (rows - 1)],
               x1_prev = prev_points[col + cols * (rows - 1)];
      compute_energy_constraint_projection(
        x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i / 2, gamma);
      con_idx++;
    }

    /* grasp position constraint */
    int vertex_idx[_agent_num] = {
      cols * rows - 1, cols * rows - rows, 0,
      cols - 1};  // hard-coded(_agent_num=4)
    for (int i = 0; i < _agent_num; i++) {
      points[vertex_idx[i]] = _measurement[i].end_pos;
    }

    /* floor collision detection */
    for (int row = 0; row < rows; row++) {
      for (int col = 0; col < cols; col++) {
        Vector3d& x = points[col + cols * row];
        if (x(2) < 0) {
          x(2) = 0.0;
        }
      }
    }
  }
  for (int i = 0; i < _vertex_num; i++) {
    _last_vels[i] = (points[i] - prev_points[i]) / dt;
  }
  _last_mesh.update_by_points(points, _measurement[0].t + dt);
  _last_mesh.rows = _mesh_param.rows;
  _last_mesh.cols = _mesh_param.cols;
  _prev_measurement = _measurement;
}

void compute_energy_constraint_projection(
  Vector3d& x0, Vector3d& x1, const Vector3d& x0_prev, const Vector3d& x1_prev,
  double& lambda, const double l, const double k, const double m,
  const double gamma) {
  double C = (x0 - x1).norm() - l;
  Matrix<double, 6, 1> dC;
  Matrix<double, 6, 1> dx;
  dC << (x0 - x1), (x1 - x0);
  dC /= (x0 - x1).norm();
  dx << x0 - x0_prev, x1 - x1_prev;
  double dLambda = (-C - lambda / k - gamma * dC.transpose() * dC) /
    ((1 + gamma) * dC.squaredNorm() / m + 1 / k);
  lambda += dLambda;
  x0 += dC.segment<3>(0) * dLambda;
  x1 += dC.segment<3>(3) * dLambda;
}

std::shared_ptr<MeshEstimator> make_mesh_estimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node) {
  return std::make_shared<MeshEstimator>(config, node);
}

/* SVK energy constraint - deprecated (too complicated) */
// Vector3<dual> compute_strain_value(
//   const Vector3<dual>& x1, const Vector3<dual>& x2, const Vector3<dual>& x3,
//   const Vector2d& X2, const Vector2d& X3) {
//   Matrix2<dual> F = compute_deformation_grad(x1, x2, x3, X2, X3);
//   Matrix2<dual> E = 0.5 * (F.transpose() * F - Matrix2<dual>::Identity());
//
//   Vector3<dual> C;
//   C << E(0, 0), E(1, 1), E(0, 1);  // [E_xx, E_yy, E_xy]
//   return C;
// }
//
// Matrix2<dual> compute_deformation_grad(
//   const Vector3<dual>& x1, const Vector3<dual>& x2, const Vector3<dual>& x3,
//   const Vector2d& X2, const Vector2d& X3) {
//   // Local basis on surface
//   Vector3<dual> e1 = (x2 - x1).normalized();
//   Vector3<dual> n = ((x2 - x1).cross(x3 - x1)).normalized();
//   Vector3<dual> e2 = n.cross(e1);  // Ensure e1, e2, n is right-handed
//
//   // Project to local 2D coordinates
//   Vector2<dual> u12, u13;
//   u12 << (x2 - x1).dot(e1), (x2 - x1).dot(e2);
//   u13 << (x3 - x1).dot(e1), (x3 - x1).dot(e2);
//
//   Matrix2<dual> Ds;
//   Ds.col(0) = u12;
//   Ds.col(1) = u13;
//
//   Matrix2d Dm;
//   Dm.col(0) = X2;
//   Dm.col(1) = X3;
//
//   return Ds * Dm.inverse();
// }

// for (int row = 0; row < rows - 1; row++) {
//   for (int col = 0; col < cols - 1; col++) {
//         /* SVK energy model - too complicated */
//         /* triangle 1 */
//         Vector3<dual> x11, x12, x13;
//         x11 = points[col + 1 + cols * row].cast<dual>();
//         x12 = points[col + cols * row].cast<dual>();
//         x13 = points[col + 1 + cols * (row + 1)].cast<dual>();
//         Vector2<dual> X11, X12, X13;
//         X11 = u_points[col + 1 + cols * row].cast<dual>();
//         X12 = u_points[col + cols * row].cast<dual>();
//         X13 = u_points[col + 1 + cols * (row + 1)].cast<dual>();
//         Matrix<double, 3, 9> dC1;
//         Vector3d C1;
//         jacobian(
//           [&](
//             const Vector3<dual>& x1, const Vector3<dual>& x2,
//             const Vector3<dual>& x3) {
//             return compute_strain_value(x1, x2, x3, X12 - X11, X13 - X11);
//           },
//           wrt(x11, x12, x13), at(x11, x12, x13), C1, dC1);
//
//
//
//         /* triangle 2 */
//         Vector3<dual> x21, x22, x23;
//         x21 = points[col + cols * (row + 1)].cast<dual>();
//         x22 = points[col + 1 + cols * (row + 1)].cast<dual>();
//         x23 = points[col + cols * row].cast<dual>();
//         Vector2<dual> X21, X22, X23;
//         X21 = u_points[col + cols * (row + 1)].cast<dual>();
//         X22 = u_points[col + 1 + cols * (row + 1)].cast<dual>();
//         X23 = u_points[col + cols * row].cast<dual>();
//         Matrix<double, 3, 9> dC2;
//         Vector3d C2;
//         jacobian(
//           [&](
//             const Vector3<dual>& x1, const Vector3<dual>& x2,
//             const Vector3<dual>& x3) {
//             return compute_strain_value(x1, x2, x3, X12 - X11, X13 - X11);
//           },
//           wrt(x21, x22, x23), at(x21, x22, x23), C2, dC2);
//
//
//   }
// }