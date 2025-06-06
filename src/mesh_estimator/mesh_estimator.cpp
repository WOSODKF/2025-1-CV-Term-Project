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
  _est_mesh.init(-1, _mesh_param.rows, _mesh_param.cols);
  _corr_mesh.init(-1, _mesh_param.rows, _mesh_param.cols);
  _prev_mesh.init(-1, _mesh_param.rows, _mesh_param.cols);
  for (int i = 0; i < _vertex_num; i++) {
    _last_vels[i].setZero();
  }

  _est_mesh_pub = make_mesh_publisher(node, "estimated", _est_mesh);
  _separate_corr_mesh = config->sim.separate_corr_mesh;
  if (_separate_corr_mesh) {
    _corr_mesh_pub = make_mesh_publisher(node, "corrected", _corr_mesh);
  } else {
    _corr_mesh_pub = nullptr;
  }

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

  std::string pkg_dir = ros::package::getPath("cv_project");
  std::string filename = pkg_dir + "/logs/mesh_timing_log.csv";
  std::cout << "file:" << filename << std::endl;
  _comp_log_file.open(filename, std::ios::out);
  if (!_comp_log_file.is_open()) {
    std::cerr << "Failed to open mesh_timing_log.csv for writing!" << std::endl;
  } else {
    _comp_log_file << "timestamp,correction_time_ms,prediction_time_ms\n";
  }
  /* TODO: assign callback to measurement subscriber? (event-based running &
   * only ros::spin() in main fn.)*/
}

/* Functions
- From received data(masks, wrenches and poses) ...
- Update mesh
- Publish data
*/

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
      // std::cout << "measurement timestamp mismatch" << std::endl;
      return;
    }
  }

  if (!_initial_mesh_ready) {
    _initial_mesh_ready = _initial_mesh_sub->initial_mesh_ready();
    if (_initial_mesh_ready) {
      _est_mesh = _initial_mesh_sub->get_data();
    } else {
      return;
    }
  }

  // 0602 checked: time sync is well matched
  if (all_ready) {
    _comp_log_file << _measure_sub[0]->get_data().t << ",";
    // publish mesh of last timestep
    if (_all_mask_ready & _xpbd_param.visual_correction_on) {
      // std::cout << "run visual correction" << std::endl;
      auto mesh_corr_start = std::chrono::steady_clock::now();
      correct_mesh();
      auto mesh_corr_fin = std::chrono::steady_clock::now();
      auto mesh_corr_dura =
        std::chrono::duration_cast<std::chrono::milliseconds>(
          mesh_corr_fin - mesh_corr_start);
      _comp_log_file << mesh_corr_dura.count() << ",";
      // std::cout << "mesh correction: " << mesh_corr_dura.count() << "ms"
      //           << std::endl;
      /* TODO: store correction time log into csv*/

      _all_mask_ready = false;
      for (int i = 0; i < _agent_num; i++) {
        _mask_ready[i] = false;
      }
    } else {
      _prev_mesh = _est_mesh;
      _comp_log_file << 0.0 << ",";
    }

    _est_mesh_pub->update(_est_mesh);
    _est_mesh_pub->pub();

    /*Test*/
    if (_separate_corr_mesh) {
      _corr_mesh_pub->update(_corr_mesh);
      _corr_mesh_pub->pub();
    }
    /*----*/

    auto mesh_pred_start = std::chrono::steady_clock::now();
    predict_mesh();
    auto mesh_pred_fin = std::chrono::steady_clock::now();
    auto mesh_pred_dura = std::chrono::duration_cast<std::chrono::milliseconds>(
      mesh_pred_fin - mesh_pred_start);
    // std::cout << "mesh prediction: " << mesh_pred_dura.count() << "ms"
    //           << std::endl;
    /* TODO: store prediction time log into csv */
    _comp_log_file << mesh_pred_dura.count() << "\n";
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
  // std::cout << "mask time: " << _mask[id].t << std::endl;
  if (all_ready) {
    for (int i = 1; i < _agent_num; i++) {
      all_ready = all_ready &&
        _mask_sub[i - 1]->get_data().t == _mask_sub[i]->get_data().t;
    }
    // if (!all_ready) {
    //   std::cout << "mask timestamp mismatch" << std::endl;
    // } else {
    //   std::cout << "mask timestamp matched" << std::endl;
    // }
  }

  _all_mask_ready = all_ready;
}

void MeshEstimator::predict_mesh() {
  const double m_i = _mesh_param.m / _vertex_num;
  const double dt = 1.0 / _measure_rate;
  const double k = _mesh_param.k;  // alpha = 1/k
  const double gamma = _mesh_param.beta / (_mesh_param.k * dt);
  const double l_0 = _mesh_param.spacing;
  const int rows = _mesh_param.rows;
  const int cols = _mesh_param.cols;

  auto points = _est_mesh.points;
  const auto prev_points = _prev_mesh.points;
  std::vector<double> lambda(_xpbd_param.strain_constraint_num, 0.0);

  /* Give predictions by external force */
  for (int i = 0; i < _vertex_num; i++) {
    points[i] += dt * _last_vels[i] - dt * dt * _g * Vector3d::UnitZ() / 2;
    // grasp points - Force is not required, if position constraint is given
  }

  /* Constraint projection */
  for (int i = 0; i < _xpbd_param.max_iter; i++) {
    compute_single_XPBD_step(
      points, prev_points, lambda, _measurement, gamma, l_0, k, m_i, rows, cols,
      _agent_num);
  }
  for (int i = 0; i < _vertex_num; i++) {
    _last_vels[i] = (points[i] - prev_points[i]) / dt;
  }
  _est_mesh.update_by_points(points, _measurement[0].t + dt);
  _est_mesh.rows = _mesh_param.rows;
  _est_mesh.cols = _mesh_param.cols;
  _prev_measurement = _measurement;
  // _prev_mesh = _est_mesh; -> skip & wait until deciding whether to run
  // correct_mesh
}

void MeshEstimator::correct_mesh() {
  /* Apply mask projection constraint */
  std::cout << "correction on " << std::endl;
  const int mesh_rows = _mesh_param.rows;
  const int mesh_cols = _mesh_param.cols;
  const int cam_height = _camera_param.height;
  const int cam_width = _camera_param.width;

  const double dt = 1.0 / _measure_rate;
  const double m_i = _mesh_param.m / _vertex_num;
  const double k = _mesh_param.k;  // alpha = 1/k
  const double gamma_E = _mesh_param.beta / (_mesh_param.k * dt);
  const double l_0 = _mesh_param.spacing;
  const double alpha_chamfer = _xpbd_param.alpha_chamfer;
  const double gamma_chamfer = _xpbd_param.beta_chamfer * alpha_chamfer / dt;

  auto points = _est_mesh.points;
  // const auto prev_points = _last_mesh.points; /// NOT THIS
  const auto prev_points = _prev_mesh.points;
  std::vector<double> lambda_xpbd(_xpbd_param.strain_constraint_num, 0.0);
  std::vector<double> lambda_chamfer(_agent_num * _vertex_num, 0.0);

  /* convert mask into cv::Mat & perform distance transform */
  std::vector<cv::Mat> cv_mask, dist_transform, labels;
  cv_mask.resize(_agent_num);
  dist_transform.resize(_agent_num);
  labels.resize(_agent_num);
  for (int id = 0; id < _agent_num; id++) {
    cv_mask[id] = cv::Mat(
      _mask[id].data.rows(), _mask[id].data.cols(), CV_8UC1,
      (void*)_mask[id].data.data());
    cv::distanceTransform(
      cv_mask[id], dist_transform[id], labels[id], cv::DIST_L2, 5,
      cv::DIST_LABEL_PIXEL);
  }

  for (int iter = 0; iter < _xpbd_param.correction_iter; iter++) {
    /* modified champfer distance constraint */
    int chamfer_idx = 0;
    for (int id = 0; id < _agent_num; id++) {
      auto cam_pos = _prev_measurement[id].cam_pos;
      auto cam_rot = _prev_measurement[id].cam_rot;
      auto cam_P = _camera_param.compute_P(cam_pos, cam_rot);

      for (int row = 0; row < mesh_rows; row++) {
        for (int col = 0; col < mesh_cols; col++) {
          /* Projection to each view */
          Vector3d& x = points[col + mesh_cols * row];
          const Vector3d x_prev = prev_points[col + mesh_cols * row];
          Vector4d hom_pos;
          hom_pos << x, 1;
          Vector3d proj_pos = cam_P * hom_pos;
          Vector3d proj_pos_normalized;
          if (std::abs(proj_pos(2)) > 1e-4) {
            proj_pos_normalized = proj_pos / proj_pos(2);  // col, row, 1
          } else {
            // std::cout << "filtered(|z| < 1e-4): " << proj_pos.transpose()
            //           << std::endl;
            chamfer_idx++;
            continue;
          }
          int proj_col = static_cast<int>(std::round(proj_pos_normalized(0)));
          int proj_row = static_cast<int>(std::round(proj_pos_normalized(1)));
          if (
            proj_col >= cam_width || proj_col < 0 || proj_row >= cam_height ||
            proj_row < 0) {
            // std::cout << "filtered(out of view): "
            //           << proj_pos_normalized.transpose() << std::endl;
            chamfer_idx++;
            continue;
          }
          if (_mask[id].data(proj_row, proj_col) == 1) {
            // std::cout << "filtered(inside the mask): "
            //           << proj_pos_normalized.transpose() << std::endl;
            chamfer_idx++;
            continue;
          }

          /* chamfer distance */
          /* approx nearest pixel and distance from distance transform */
          CV_Assert(labels[id].type() == CV_32SC1);
          int flat_idx = labels[id].at<int>(proj_row, proj_col);
          Vector2d target_point(
            flat_idx % cam_width, flat_idx / cam_width);  // (x, y) = (col, row)
          // std::cout << "non-overlapping point detected:"
          //           << proj_pos_normalized.segment<2>(0).transpose()
          //           << std::endl;
          /* grad p */
          double t_ki = proj_pos(2);
          Vector3d dt_ki = cam_P.bottomLeftCorner(1, 3).transpose();
          Matrix<double, 2, 3> dp_ki = cam_P.topLeftCorner(2, 3) / t_ki -
            proj_pos_normalized.segment<2>(0) / t_ki * dt_ki.transpose();

          /* constraint value & gradient */
          double C =
            (proj_pos_normalized.segment<2>(0) - target_point).squaredNorm();
          Vector3d dC = 2 * dp_ki.transpose() *
            (proj_pos_normalized.segment<2>(0) - target_point);
          double dLambda = (-C - alpha_chamfer * lambda_chamfer[chamfer_idx] -
                            gamma_chamfer * dC.transpose() * (x - x_prev)) /
            ((1 + gamma_chamfer) / m_i * dC.squaredNorm() + alpha_chamfer);
          x += dC * dLambda / m_i;
          if (!(dC * dLambda / m_i).isZero()) {
            std::cout << "C:" << C << std::endl;
            std::cout << "x - x_prev: " << x - x_prev << std::endl;
            std::cout << "target_point: " << target_point << std::endl;
            std::cout << "proj_pos: " << proj_pos_normalized.segment<2>(0)
                      << std::endl;
            std::cout << "dx:" << dC * dLambda / m_i << std::endl;
          }
          chamfer_idx++;
        }
      }
    }
    /* run XPBD step for physical consistency */
    // if (iter % 5 == 0) {
    //   compute_single_XPBD_step(
    //     points, prev_points, lambda_xpbd, _prev_measurement, gamma_E, l_0, k,
    //     m_i, mesh_rows, mesh_cols, _agent_num);
    // }
  }

  /* update velocities */
  for (int i = 0; i < _vertex_num; i++) {
    _last_vels[i] = (points[i] - prev_points[i]) / dt;
  }
  if (_separate_corr_mesh) {
    _corr_mesh.update_by_points(points, _prev_measurement[0].t);
    _corr_mesh.cols = _mesh_param.cols;
    _corr_mesh.rows = _mesh_param.rows;
    _prev_mesh = _corr_mesh;
  } else {
    _est_mesh.update_by_points(points, _prev_measurement[0].t);
    _est_mesh.rows = _mesh_param.rows;
    _est_mesh.cols = _mesh_param.cols;
    _prev_mesh = _est_mesh;
  }
}

/* XBPD computation */
void compute_single_XPBD_step(
  std::vector<Vector3d>& points, const std::vector<Vector3d>& prev_points,
  std::vector<double>& lambda,
  const std::vector<robot_measurement_t>& measurement, const double gamma,
  const double l_0, const double k, const double m_i, const int rows,
  const int cols, const int agent_num) {
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
        x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i, gamma);
      con_idx++;

      /* diagonal */
      compute_energy_constraint_projection(
        x0, x2, x0_prev, x2_prev, lambda[con_idx], sqrt(2) * l_0, k, m_i,
        gamma);
      con_idx++;

      /* orthogonal 2 */
      compute_energy_constraint_projection(
        x0, x3, x0_prev, x3_prev, lambda[con_idx], l_0, k, m_i, gamma);
      con_idx++;

      // /* diagonal 2? */
      // compute_energy_constraint_projection(
      //   x1, x3, x1_prev, x3_prev, lambda[con_idx], sqrt(2) * l_0, k, m_i /
      //   2, gamma);
      // con_idx++;
    }
  }
  for (int row = 0; row < rows - 1; row++) {
    /* vertical edge */
    Vector3d& x0 = points[cols - 1 + cols * row];
    Vector3d& x1 = points[cols - 1 + cols * (row + 1)];
    Vector3d x0_prev = prev_points[cols - 1 + cols * row],
             x1_prev = prev_points[cols - 1 + cols * (row + 1)];
    compute_energy_constraint_projection(
      x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i, gamma);
    con_idx++;
  }
  for (int col = 0; col < cols - 1; col++) {
    /* horizontal edge */
    Vector3d& x0 = points[col + cols * (rows - 1)];
    Vector3d& x1 = points[col + 1 + cols * (rows - 1)];
    Vector3d x0_prev = prev_points[col + cols * (rows - 1)],
             x1_prev = prev_points[col + 1 + cols * (rows - 1)];
    compute_energy_constraint_projection(
      x0, x1, x0_prev, x1_prev, lambda[con_idx], l_0, k, m_i, gamma);
    con_idx++;
  }

  /* grasp position constraint */
  int vertex_idx[agent_num] = {
    cols * rows - 1, cols * rows - rows, 0,
    cols - 1};  // hard-coded(_agent_num=4)
  for (int i = 0; i < agent_num; i++) {
    points[vertex_idx[i]] = measurement[i].end_pos;
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
  double dLambda = (-C - lambda / k - gamma * dC.transpose() * dx) /
    ((1 + gamma) * dC.squaredNorm() / m * 2 + 1 / k);
  lambda += dLambda;
  x0 += dC.segment<3>(0) * dLambda / m;
  x1 += dC.segment<3>(3) * dLambda / m;
}

std::shared_ptr<MeshEstimator> make_mesh_estimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node) {
  return std::make_shared<MeshEstimator>(config, node);
}

// /* Chamfer distance computation */
// void compute_single_chamfer_step(
//   std::vector<Vector3d>& points, std::vector<double>& lambda_chamfer,
//   int& chamfer_idx, const std::vector<Vector3d>& prev_points,
//   const std::vector<robot_measurement_t> prev_measurement,
//   const camera_param_t& camera_param, const std::vector<cv::Mat>& cv_mask,
//   const std::vector<cv::Mat>& labels,
//   const std::vector<cv::Mat>& dist_transform, const int agent_num,
//   const int mesh_rows, const int mesh_cols, const double alpha_chamfer,
//   const double gamma_chamfer, const double m_i
// ) {
//   /* MIGRATE TO compute_single_chamfer_step */
//   int chamfer_idx = 0;
//   for (int id = 0; id < agent_num; id++) {
//     auto cam_pos = prev_measurement[id].cam_pos;
//     auto cam_rot = prev_measurement[id].cam_rot;
//     auto cam_P = camera_param.compute_P(cam_pos, cam_rot);
//     int cam_width = camera_param.width;
//     int cam_height = camera_param.height;
//
//     for (int row = 0; row < mesh_rows; row++) {
//       for (int col = 0; col < mesh_cols; col++) {
//         /* Projection to each view */
//         Vector3d& x = points[col + mesh_cols * row];
//         const Vector3d x_prev = prev_points[col + mesh_cols * row];
//         Vector4d hom_pos;
//         hom_pos << x, 1;
//         Vector3d proj_pos = cam_P * hom_pos;
//         Vector3d proj_pos_normalized;
//         if (proj_pos(2) > 0.01) {
//           proj_pos_normalized = proj_pos / proj_pos(2);
//         } else {
//           chamfer_idx++;
//           continue;
//         }
//         int proj_col = static_cast<int>(std::round(proj_pos_normalized(0)));
//         int proj_row = static_cast<int>(std::round(proj_pos_normalized(1)));
//         if (
//           proj_col >= cam_width || proj_col < 0 || proj_row >= cam_height ||
//           proj_row < 0) {
//           chamfer_idx++;
//           continue;
//         }
//         if (cv_mask[id].data[proj_row, proj_col] == 1) {
//           chamfer_idx++;
//           continue;
//         }
//
//         /* chamfer distance */
//         /* approx nearest pixel and distance from distance transform */
//         // float dist = dist_transform[id].at<float>(proj_row, proj_col);
//         int flat_idx = labels[id].at<int>(proj_row, proj_col);
//         Vector2d target_point(flat_idx / cam_width, flat_idx % cam_width);
//
//         /* grad p */
//         double t_ki = proj_pos(2);
//         Vector3d dt_ki = cam_P.bottomLeftCorner(1, 3).transpose();
//         Matrix<double, 2, 3> dp_ki = cam_P.topLeftCorner(2, 3) / t_ki -
//           proj_pos_normalized.segment<2>(0) / t_ki * dt_ki.transpose();
//
//         /* constraint value & gradient */
//         double C =
//           (proj_pos_normalized.segment<2>(0) - target_point).squaredNorm();
//         Vector3d dC =
//           2 * dp_ki.transpose() * (proj_pos.segment<2>(0) - target_point);
//         double dLambda = (-C - alpha_chamfer * lambda_chamfer[chamfer_idx] -
//                           gamma_chamfer * dC.transpose() * (x - x_prev)) /
//           ((1 + gamma_chamfer) / m_i * dC.squaredNorm() + alpha_chamfer);
//         x += dC * dLambda / m_i;
//         // if (!(dC * dLambda / m_i).isZero()) {
//         //   std::cout << "C:" << C << std::endl;
//         //   std::cout << "x - x_prev: " << x - x_prev << std::endl;
//         //   std::cout << "target_point: " << target_point << std::endl;
//         //   std::cout << "proj_pos: " << proj_pos_normalized.segment<2>(0)
//         //             << std::endl;
//         //   std::cout << "dx:" << dC * dLambda / m_i << std::endl;
//         // }
//         chamfer_idx++;
//       }
//     }
//   }
// }
//
// void compute_chamfer_distance_projection(
//   Vector3d& x, double& lambda, const robot_measurement_t& measurement,
//   const camera_param_t& camera_param, const mask_data_t& mask) {
// }

// MatrixXi projection;
// projection.resize(_camera_param.height, _camera_param.width);
// for (int i = 0; i < _agent_num; i++) {
//   projection.setZero();
//   auto cam_pos = _prev_measurement[i].cam_pos;
//   auto cam_rot = _prev_measurement[i].cam_rot;
//   auto cam_P = _camera_param.compute_P(cam_pos, cam_rot);
//   for (int row = 0; row < rows; row++) {
//     for (int col = 0; col < cols; col++) {
//       Vector4d hom_pos;
//       hom_pos << points[col + cols * row], 1;
//       Vector3d proj_pos = cam_P * hom_pos;
//       if (proj_pos(2) > 0.01) {
//         proj_pos = proj_pos / proj_pos(2);
//       } else {
//         continue;
//       }
//       int proj_col = static_cast<int>(std::round(proj_pos(0)));
//       int proj_row = static_cast<int>(std::round(proj_pos(1)));
//       if (
//         proj_col > cam_width || proj_col < 0 || proj_row > cam_height ||
//         proj_row < 0) {
//         continue;
//       }
//       projection(proj_row, proj_col) = 1;
//     }
//   }
//   /* compute modified chamfer distance */
// }

/* minimal XPBD with _prev_measurement */

/* SVK energy constraint - deprecated (too complicated) */
// Vector3<dual> compute_strain_value(
//   const Vector3<dual>& x1, const Vector3<dual>& x2, const Vector3<dual>&
//   x3, const Vector2d& X2, const Vector2d& X3) { Matrix2<dual> F =
//   compute_deformation_grad(x1, x2, x3, X2, X3); Matrix2<dual> E = 0.5 *
//   (F.transpose() * F - Matrix2<dual>::Identity());
//
//   Vector3<dual> C;
//   C << E(0, 0), E(1, 1), E(0, 1);  // [E_xx, E_yy, E_xy]
//   return C;
// }
//
// Matrix2<dual> compute_deformation_grad(
//   const Vector3<dual>& x1, const Vector3<dual>& x2, const Vector3<dual>&
//   x3, const Vector2d& X2, const Vector2d& X3) {
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

// nanoflann_adaptor::NNResult find_nearest_neighbor(
//   const Vector2d& query, const mask_data_t& mask) {
//   std::cout << "HHHHHH" << std::endl;
//   std::cout << "t=" << mask.t << std::endl;
//   std::vector<Eigen::Vector2d> foreground_pixels;
//   for (int y = 0; y < mask.data.rows(); y++) {
//     for (int x = 0; x < mask.data.cols(); x++) {
//       if (mask.data(y, x) > 0) {
//         foreground_pixels.emplace_back(x, y);  // (u, v)
//       }
//     }
//   }
//
//   nanoflann_adaptor::PointCloud cloud;
//   cloud.pts = foreground_pixels;
//   nanoflann_adaptor::KDTree2D index(
//     2, cloud, KDTreeSingleIndexAdaptorParams(10));
//   index.buildIndex();
//
//   size_t nearest_idx;
//   double out_dist_sqr;
//   nanoflann::KNNResultSet<double> resultSet(1);
//   resultSet.init(&nearest_idx, &out_dist_sqr);
//
//   index.findNeighbors(resultSet, query.data(),
//   nanoflann::SearchParameters(10)); Eigen::Vector2d nearest_pixel =
//   cloud.pts[nearest_idx]; std::cout << "LNLLNL" << std::endl; return
//   nanoflann_adaptor::NNResult {
//     .point = nearest_pixel, .distance = out_dist_sqr};
// }

// NNResult find_nearest_neighbor(
//   const Vector2d& query, const mask_data_t& mask, const int height,
//   const int width) {
//   std::cout << "NN: t=" << mask.t << std::endl;
//   const int d = 2;  // data dim
//   const int M =
//     16;  // HNSW connectivity (higher = better recall, slower index build)
//   faiss::IndexHNSWFlat index(d, M);
//
//   std::vector<float> data_flatten;
//   std::vector<std::pair<int, int>> id_to_uv;  // to store (u, v) mapping
//   data_flatten.reserve(2 * height * width);
//   id_to_uv.reserve(height * width);
//   for (int y = 0; y < mask.data.rows(); y++) {
//     for (int x = 0; x < mask.data.cols(); x++) {
//       if (mask.data(y, x) > 0) {
//         data_flatten.push_back(static_cast<float>(x));  // u
//         data_flatten.push_back(static_cast<float>(y));
//         id_to_uv.push_back({x, y});
//       }
//     }
//   }
//   int n_data = data_flatten.size() / d;
//
//   // Optional tuning: higher efConstruction → better accuracy, slower build
//   index.hnsw.efConstruction = 40;
//
//   // Add your data (float pointer of size N×2)
//   index.add(n_data, data_flatten.data());
//
//   // Optional: during query, trade-off speed vs accuracy
//   index.hnsw.efSearch = 32;  // higher = better accuracy, slower
//
//   // Add data
//   index.add(n_data, data_flatten.data());
//
//   // Query point
//   std::vector<float> query_f = {
//     static_cast<float>(query(0)), static_cast<float>(query(1))};
//   int k = 1;  // number of nearest neighbors to find
//
//   std::vector<faiss::idx_t> indices(k);
//   std::vector<float> distances(k);
//
//   index.search(1, query_f.data(), k, distances.data(), indices.data());
//   //
//   //   std::cout << "Nearest neighbor index: " << indices[0] << std::endl;
//   //   std::cout << "Distance: " << distances[0] << std::endl;
//   std::cout << "NN fin" << std::endl;
//   return NNResult {
//     .distance = distances[0],
//     .point = Vector2d(id_to_uv[indices[0]].first,
//     id_to_uv[indices[0]].second)};
// }