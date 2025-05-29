#pragma once

#include "simulator/robot.hpp"
#include "utils/mesh_publisher.hpp"
#include "utils/config.hpp"
#include "utils/data_struct.hpp"

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>

class Simulator {
  public:
    Simulator(std::shared_ptr<config_t> config);
    ~Simulator();

    void run();

    void callback(const mjModel* m, mjData* d);
    void register_callback();

  private:
    static void callback_wrapper(const mjModel* m, mjData* d);
    static std::function<void(const mjModel*, mjData*)> s_callback;

    std::shared_ptr<config_t> _config;
    std::vector<std::shared_ptr<Robot>> _robot;
    std::shared_ptr<MeshPublisher> _mesh_pub;
    ros::Subscriber _segmentor_init_sub;

    mesh_data_t _mesh;

    int _agent_num;
};
