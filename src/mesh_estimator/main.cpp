#include "mesh_estimator/mesh_estimator.hpp"
#include "utils/config.hpp"

#include <ros/ros.h>
#include "ros/console.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mesh_estimator_node");

  auto config = get_config();
//   auto mesh_estimator = make_mesh_estimator(config);

  ros::spin();

  return 1;
}