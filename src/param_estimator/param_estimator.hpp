#pragma once

#include "param_estimator/ros_io/error_subscriber.hpp"
#include "param_estimator/ros_io/param_publisher.hpp"

#include "utils/config.hpp"
#include "utils/data_struct.hpp"
#include "utils/math.hpp"

#include <Eigen/Dense>

class ParamEstimator {
public:
  ParamEstimator(std::shared_ptr<config_t> config);
  void run();

private:
};

std::shared_ptr<ParamEstimator> make_param_estimator(
  std::shared_ptr<config_t> config);