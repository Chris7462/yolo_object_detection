#pragma once

// C++ header
#include <queue>
//  #include <mutex>
//  #include <memory>
//  #include <vector>
//  #include <string>

//  // openCV header
//  //#include <opencv2/opencv.hpp>
//  #include <opencv2/core.hpp>
//  #include <opencv2/dnn.hpp>

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

//  bool load_classes(fs::path class_file);
//  void load_net(fs::path model_file);
//  cv::Mat format_yolov5(const cv::Mat & source);
//  void detect(cv::Mat & image, std::vector<Detection> & output);

//  std::vector<std::string> class_list_;
//  cv::dnn::Net net_;
};

} // namespace yolo_object_detection
