#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>

// openCV header
#include <opencv2/opencv.hpp>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


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

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;

  std::mutex mtx_;

  bool load_classes(fs::path class_file);
  void load_net(fs::path model_file);

  std::vector<std::string> class_list_;
  cv::dnn::Net net_;
};

}
