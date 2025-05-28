#pragma once

#include "mesh_estimator/ros_io/error_publisher.hpp"
#include "mesh_estimator/ros_io/param_subscriber.hpp"
#include "mesh_estimator/ros_io/mask_subscriber.hpp"
#include "mesh_estimator/ros_io/measurement_subscriber.hpp"

#include "utils/data_struct.hpp"
#include "utils/math.hpp"
#include "utils/config.hpp"
#include "utils/mesh_publisher.hpp"

#include <Eigen/Dense>
#include <vector>

class MeshEstimator {
public:
  MeshEstimator(std::shared_ptr<config_t> config, ros::NodeHandle& node);
  void run();

private:
  int _agent_num;
//   int _rate;
  camera_param_t _camera_param;

  std::vector<mask_data_t> _mask;
  std::vector<robot_measurement_t> _measurement;
  est_mesh_param_t _mesh_param;
  mesh_data_t _est_mesh;
  mesh_data_t _GT_mesh;

  std::shared_ptr<ErrorPublisher> _err_pub;
  std::shared_ptr<ParamSubscriber> _param_sub;
  std::vector<std::shared_ptr<MaskSubscriber>> _mask_sub;
  std::vector<std::shared_ptr<MeasurementSubscriber>> _measure_sub;

  void update_mesh();
  void update_data();
  void publish_data();
};

std::shared_ptr<MeshEstimator> make_mesh_estimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node);