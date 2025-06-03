#pragma once

#include "mesh_estimator/ros_io/error_publisher.hpp"
#include "mesh_estimator/ros_io/param_subscriber.hpp"
#include "mesh_estimator/ros_io/mask_subscriber.hpp"
#include "mesh_estimator/ros_io/measurement_subscriber.hpp"
#include "mesh_estimator/ros_io/initial_mesh_subscriber.hpp"

#include "utils/data_struct.hpp"
#include "utils/math.hpp"
#include "utils/config.hpp"
#include "utils/mesh_publisher.hpp"

#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <chrono>

using namespace Eigen;

class MeshEstimator {
public:
  MeshEstimator(std::shared_ptr<config_t> config, ros::NodeHandle& node);
  void run();

private:
  int _agent_num;
  int _vertex_num;
  double _g;  // gravity
  double _measure_rate;
  double _view_rate;
  camera_param_t _camera_param;
  XPBD_param_t _xpbd_param;

  std::vector<mask_data_t> _mask;
  std::vector<robot_measurement_t> _measurement;
  std::vector<robot_measurement_t> _prev_measurement;
  est_mesh_param_t _mesh_param;
  undeformed_mesh_t _undeformed_mesh;
  mesh_data_t _est_mesh;
  mesh_data_t _prev_mesh;
  mesh_data_t _GT_mesh;
  std::vector<Vector3d> _last_vels;  // mesh vertex velocities

  std::shared_ptr<ErrorPublisher> _err_pub;
  std::shared_ptr<ParamSubscriber> _param_sub;
  std::shared_ptr<MeshPublisher> _mesh_pub;
  std::shared_ptr<InitialMeshSubscriber> _initial_mesh_sub;
  std::vector<std::shared_ptr<MaskSubscriber>> _mask_sub;
  std::vector<std::shared_ptr<MeasurementSubscriber>> _measure_sub;
  std::vector<bool> _measurement_ready;
  std::vector<bool> _mask_ready;
  bool _all_mask_ready;
  bool _initial_mesh_ready;

  void update_mesh();
  void publish_data();

  void correct_mesh();
  void compute_XPBD();

  void measure_receiving_callback(const std::string& frame_id);
  void mask_receiving_callback(const std::string& frame_id);
};

std::shared_ptr<MeshEstimator> make_mesh_estimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node);

/* XPBD computation */
void compute_single_XPBD_step(
  std::vector<Vector3d>& points, const std::vector<Vector3d>& prev_points,
  std::vector<double>& lambda,
  const std::vector<robot_measurement_t>& measurement, const double gamma,
  const double l_0, const double k, const double m_i, const int rows,
  const int cols, const int _agent_num);

void compute_energy_constraint_projection(
  Vector3d& x0, Vector3d& x1, const Vector3d& x0_prev, const Vector3d& x1_prev,
  double& lambda, const double l, const double k, const double m,
  const double gamma);

/* Chamfer distance computation */
void compute_single_chamfer_step();
void compute_chamfer_distance_projection();
// nanoflann_adaptor::NNResult find_nearest_neighbor(const Vector2d& query,
// const mask_data_t& mask);

struct NNResult {
  double distance;
  Vector2d point;
};
NNResult find_nearest_neighbor(
  const Vector2d& query, const mask_data_t& mask, const int height,
  const int width);