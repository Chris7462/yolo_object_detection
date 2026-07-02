#include <memory>

// ROS header
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// local header
#include "yolo_object_detection/yolo_object_detection.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<yolo_object_detection::YoloObjectDetection>();

  rclcpp::executors::EventsCBGExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting YOLO Object Detection with EventsCBGExecutor");

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
