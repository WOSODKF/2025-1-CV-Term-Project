#pragma once

#include <cv_project/robotView.h>
#include "utils/data_struct.hpp"

#include "mujoco/mujoco.h"

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>


class ViewPublisher {
public:
  ViewPublisher(ros::NodeHandle& node, int agent_ID);
  void pub();

  void update_view(
    const mjModel* m, mjData* d, mjvOption& opt, mjvCamera& cam,
    mjvScene& scn, mjrContext& con, const mjrRect& viewport, int cam_ID);

private:
  ros::Publisher _data_pub;
  int _agent_ID;

  cv_project::robotView _last_view_msg;
};

std::shared_ptr<ViewPublisher> make_view_publisher(
  ros::NodeHandle node, int agent_ID);