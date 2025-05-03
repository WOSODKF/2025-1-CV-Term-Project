#include "simulator/simulator.hpp"
#include "utils/config.hpp"

#include <ros/ros.h>
#include "ros/console.h"

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

#include <thread>

std::function<void(const mjModel*, mjData*)> Simulator::s_callback = nullptr;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");

  auto config = get_config();
  auto simulator = Simulator(config);

  // Run ROS communication loop in a separate thread
  std::thread ros_thread([&]() { ros::spin(); });

  try {
    simulator.run();
  } catch (const std::exception& e) {
    ROS_ERROR("Simulator error: %s", e.what());
  }

  ros_thread.join();

  return 1;
}