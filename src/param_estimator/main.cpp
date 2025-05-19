#include "param_estimator/param_estimator.hpp"
#include "utils/config.hpp"

#include <ros/ros.h>
#include "ros/console.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "param_estimator_node");

  auto config = get_config();
//   auto param_estimator = make_param_estimator(config);

  ros::spin();

  return 1;
}