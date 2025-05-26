#pragma once

#include "utils/config.hpp"
#include "utils/data_struct.hpp"
#include "utils/math.hpp"
#include "simulator/ros_io/setpoint_subscriber.hpp"
#include "simulator/ros_io/measurement_publisher.hpp"
#include "simulator/ros_io/state_publisher.hpp"

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <memory>
#include <functional>
#include <filesystem>
#include <chrono>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>

class Robot {
public:
  Robot(
    const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
    ros::NodeHandle& node, int agent_ID);

  const mujoco_control_t inverse_kinematics(
    const setpoint_t& setpoint, double dt);

  void compute_control(const mjModel* m, const mjData* d);
  void set_mujoco_control(const mjModel* m, mjData* d);

  void update_view(
    const mjModel* m, mjData* d, mjvOption& opt, mjvCamera& cam, mjvScene& scn,
    mjrContext& con, const mjrRect& viewport, bool data_gen_mode);
  void update_wrench(const mjModel* m, mjData* d);
  void publish_state();
  void publish_measurement();

  const mujoco_control_t get_control() const;
  const mujoco_control_id_t get_id() const;

private:
  int _agent_ID;
  bool _render_view;
  std::string _dataset_code;

  robot_IK_param_t _ik_param;  // in config
  robot_FK_param_t _fk_param;
  robot_state_t _current_state;
  mujoco_control_id_t _mj_ID;
  mujoco_control_t _last_control;
  mujoco_robot_wrench_t _last_wrench;
  cv::Mat _last_view;

  std::shared_ptr<SetpointSubscriber> _setpoint_sub;
  std::shared_ptr<MeasurementPublisher> _measurement_pub;
  std::shared_ptr<StatePublisher> _state_pub;

  const fk_result_t forward_kinematics(const mujoco_control_t& theta);
};

std::shared_ptr<Robot> make_robot(
  const mjModel* m, const mjData* d, std::shared_ptr<config_t> config,
  ros::NodeHandle& node, int agent_ID);