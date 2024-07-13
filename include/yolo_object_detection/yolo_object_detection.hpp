#pragma once

// C++ header
#include <queue>
#include <mutex>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// local header
#include "yolo_inference/inference.hpp"

namespace yolo_object_detection
{

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

  yolo::Inference inference_;
};

} // namespace yolo_object_detection
