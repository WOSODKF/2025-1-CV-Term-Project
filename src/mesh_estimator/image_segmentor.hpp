
// Chat GPT snipet for segmentation
// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// 
// ros::Publisher mask_pub;
// 
// void image_callback(const sensor_msgs::ImageConstPtr& msg) {
//   try {
//     // Convert to cv::Mat
//     cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
//     cv::Mat gray, thresh, mask;
// 
//     // Convert to grayscale
//     cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
// 
//     // Threshold to segment bright (white) cloth
//     cv::threshold(gray, thresh, 200, 255, cv::THRESH_BINARY);
// 
//     // Morphological cleanup (remove noise)
//     cv::Mat kernel =
//       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
//     cv::morphologyEx(thresh, mask, cv::MORPH_CLOSE, kernel);
//     cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
// 
//     // Optional: visualize
//     cv::imshow("cloth mask", mask);
//     cv::waitKey(1);
// 
//     // If you want to republish the mask as an Image:
//     // sensor_msgs::ImagePtr mask_msg = cv_bridge::CvImage(msg->header, "mono8",
//     // mask).toImageMsg(); mask_pub.publish(mask_msg);
// 
//   } catch (cv_bridge::Exception& e) {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }
// }
// 
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "cloth_segmentor_node");
//   ros::NodeHandle nh;
// 
//   ros::Subscriber sub = nh.subscribe("/simulator/view_0", 1, image_callback);
//   // mask_pub = nh.advertise<sensor_msgs::Image>("/cloth_mask", 1);  // optional
// 
//   ros::spin();
//   return 0;
// }
