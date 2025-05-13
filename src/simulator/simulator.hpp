#pragma once

#include "simulator/robot.hpp"
// #include "simulator/ros_io/setpoint_subscriber.hpp"
#include "simulator/ros_io/data_publisher.hpp"
#include "utils/config.hpp"
#include "utils/data_struct.hpp"

// #include "mujoco.h"  // mujoco-3.2.0
#include "mujoco/mujoco.h"  // mujoco-3.3.1
#include <GLFW/glfw3.h>

#include <ros/ros.h>
#include "ros/console.h"

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <functional>
#include <optional>

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

    void set_eq_data(mjModel* m, mjData* d);

    std::shared_ptr<config_t> _config;

    std::vector<std::shared_ptr<Robot>> _robot;

    // std::vector<std::shared_ptr<SetpointSubscriber>> _setpoint_sub;
    std::vector<std::shared_ptr<DataPublisher>> _data_pub;
    // std::vector<mujoco_control_id_t> _mj_ID;
    // std::vector<mujoco_control_t> _last_control;

    int _agent_num;
  };
