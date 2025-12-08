#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include "libtrossen_arm/trossen_arm.hpp"

class EmptyNode : public rclcpp::Node
{
public:
  EmptyNode() : Node("empty_ros_node")
  {
    RCLCPP_INFO(get_logger(), "EmptyNode constructed");
  }
};

int main(int argc, char ** argv)
{
  std::cout << "empty_ros_node: before init\n";
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<EmptyNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  std::cout << "empty_ros_node: after shutdown\n";
  return 0;
}
