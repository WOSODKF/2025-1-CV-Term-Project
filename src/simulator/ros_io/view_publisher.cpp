#include "simulator/ros_io/view_publisher.hpp"

ViewPublisher::ViewPublisher(ros::NodeHandle& node, int agent_ID) {
  std::string topic_name = "view_" + std::to_string(agent_ID);
  _data_pub = node.advertise<cv_project::robotView>(topic_name, 32);
  // TODO: initialize _last_data_msg
}

void ViewPublisher::pub() {
  _data_pub.publish(_last_view_msg);
}

void ViewPublisher::update_view(
  const mjModel* m, mjData* d, mjvOption& opt, mjvCamera& cam, mjvScene& scn,
  mjrContext& con, const mjrRect& viewport, int cam_ID) {
  const int WIDTH = viewport.width;
  const int HEIGHT = viewport.height;
  unsigned char rgb_buffer[WIDTH * HEIGHT * 3];

  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = cam_ID;

  // Update scene
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

  // Render
  mjr_render(viewport, &scn, &con);

  // Read pixels (RGB only here)
  mjr_readPixels(rgb_buffer, NULL, viewport, &con);

  /*-------HAVE TO CHECK(250519)--------------*/

  // Convert to OpenCV BGR (MuJoCo outputs bottom-to-top RGB)
  cv::Mat rgb_img(HEIGHT, WIDTH, CV_8UC3, rgb_buffer);
  cv::Mat rgb_flipped, bgr_img;
  cv::flip(rgb_img, rgb_flipped, 0);
  cv::cvtColor(rgb_flipped, bgr_img, cv::COLOR_RGB2BGR);

  // Publish (HAVE TO CHECK)
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_img).toImageMsg();
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "camera_frame_" + std::to_string(cam_ID);
//   image_pub.publish(msg);

}

std::shared_ptr<ViewPublisher> make_view_publisher(
  ros::NodeHandle node, int agent_ID) {
  return std::make_shared<ViewPublisher>(node, agent_ID);
}