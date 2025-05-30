#include "mesh_estimator/mesh_estimator.hpp"
#include "utils/config.hpp"

#include <ros/ros.h>
#include "ros/console.h"

#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mesh_estimator_node");
  ros::NodeHandle node("~");

  auto config = get_config();
  auto mesh_estimator = make_mesh_estimator(config, node);

  std::thread ros_thread([&]() { ros::spin(); });  // for event-based operation: run spin() on another thread

  /* Note: all times should be controlled by mujoco sim time -> asynchronized operation */

  try {
    mesh_estimator->run();
  } catch (const std::exception& e) {
    ROS_ERROR("Estimator error: %s", e.what());
  }

  ros_thread.join();

  return 1;
}