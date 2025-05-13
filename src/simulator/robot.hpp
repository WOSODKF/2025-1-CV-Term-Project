#pragma once

#include "utils/config.hpp"
#include "utils/data_struct.hpp"
#include "utils/math.hpp"
#include "simulator/ros_io/setpoint_subscriber.hpp"

#include "mujoco/mujoco.h"

#include <Eigen/Dense>
#include <memory>
#include <functional>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

class Robot {
public:
  Robot(
    const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
    ros::NodeHandle& node, int agent_ID);

  const mujoco_control_t inverse_kinematics(
    const setpoint_t& setpoint, double dt);
  const mujoco_control_t get_control() const;
  const mujoco_control_id_t get_id() const;
  void compute_control(const mjModel* m, const mjData* d);
  void set_mujoco_control(const mjModel* m, mjData* d);

private:
  int _agent_ID;
  robot_FK_param_t _fk_param;
  robot_state_t _current_state;
  mujoco_control_id_t _mj_ID;
  mujoco_control_t _last_control;

  std::shared_ptr<SetpointSubscriber> _setpoint_sub;

  const fk_result_t forward_kinematics(const mujoco_control_t& theta);
};

std::shared_ptr<Robot> make_robot(
  const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
  ros::NodeHandle& node, int agent_ID);