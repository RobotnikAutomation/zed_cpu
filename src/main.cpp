#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto zed_camera_node = std::make_shared<zed_cpu::ZedCameraNode>();
  rclcpp::Rate rate(30);
  zed_camera_node->InitCameraInfo();
  while (rclcpp::ok()) {
    zed_camera_node->run();
    rclcpp::spin_some(zed_camera_node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
