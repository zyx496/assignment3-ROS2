#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hikrobot_camera/camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hikrobot_camera::CameraNode>();
  rclcpp::spin(node);
  node.reset();          // 先析构节点：停线程、停取流、关闭相机
  rclcpp::shutdown();
  return 0;
}
