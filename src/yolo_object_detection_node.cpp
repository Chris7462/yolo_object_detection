#include "yolo_object_detection/yolo_object_detection.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yolo_object_detection::YoloObjectDetection>());
  rclcpp::shutdown();
  return 0;
}
