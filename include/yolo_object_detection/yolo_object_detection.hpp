#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


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

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;

  std::mutex mtx_;
};

}
