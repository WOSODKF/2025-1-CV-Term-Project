#include "mesh_estimator/mesh_estimator.hpp"

MeshEstimator::MeshEstimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node) {
  if (node.getParam("agent_num", _agent_num)) {
  } else {
    ROS_ERROR("/mesh_estimator/agent_num is not found");
  }
  _camera_param = config->camera;

  _mask = std::vector<mask_data_t>(_agent_num);
  _measurement = std::vector<robot_measurement_t>(_agent_num);

  _mask_sub.resize(_agent_num);
 // = std::vector<std::shared_ptr<MaskSubscriber>>(_agent_num);
  _measure_sub.resize(_agent_num);
  // = std::vector<std::shared_ptr<MeasurementSubscriber>>(_agent_num);

  for (int i = 0; i < _agent_num; i++) {
    _mask_sub[i] = make_mask_subscriber(node, config, i);
    _measure_sub[i] = make_measurement_subscriber(node, i);
  }
}

void MeshEstimator::run() {
  while (ros::ok()) {
    /* main loop */

    /* TODO: delay until seg node and mesh initialized (receive init message from seg node) s*/
    update_data();
    update_mesh();
    // publish_data();
  }
}

/* Functions
- From received data(masks, wrenches and poses) ...
- Update mesh
- Publish data
*/

void MeshEstimator::update_mesh() {
  /* TODO: run PBD / if mask received - run correction */

}

void MeshEstimator::update_data() {
  // update data to recently received data
  for (int i = 0; i < _agent_num; i++) {
    _mask[i] = _mask_sub[i]->get_data();
    // _measurement[i] = _measure_sub[i]->get_data();
  }
  // _mesh_param = _param_sub->get_data();
}

void MeshEstimator::publish_data() {
  //   _err_pub->update();
  // _err_pub->pub();
}

std::shared_ptr<MeshEstimator> make_mesh_estimator(
  std::shared_ptr<config_t> config, ros::NodeHandle& node) {
  return std::make_shared<MeshEstimator>(config, node);
}