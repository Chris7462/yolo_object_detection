#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <filesystem>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV header
#include <opencv2/dnn.hpp>

// local header
// #include "yolo_inference/inference.hpp"

namespace yolo_object_detection
{

namespace fs = std::filesystem;

class YoloObjectDetection : public rclcpp::Node
{
public:
  YoloObjectDetection();
  ~YoloObjectDetection() = default;

private:
  void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  void timer_callback();
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;

  std::mutex mtx_;

  bool get_classes(std::string classes_file);

  std::vector<std::string> classes_;
  void load_net(fs::path model_file);

  cv::dnn::Net net_;
};

} // namespace yolo_object_detection
