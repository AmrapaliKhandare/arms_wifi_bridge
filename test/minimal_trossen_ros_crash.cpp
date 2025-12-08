#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "libtrossen_arm/trossen_arm.hpp"

int main(int argc, char ** argv)
{
  std::cout << "minimal_trossen_ros_crash: before init" << std::endl;

  // This *alone* is enough to trigger the crash in your setup.
  rclcpp::init(argc, argv);

  // Construct + immediately destroy a driver (we probably won't even reach here).
  std::cout << "minimal_trossen_ros_crash: after init, constructing driver" << std::endl;
  trossen_arm::TrossenArmDriver driver;
  std::cout << "minimal_trossen_ros_crash: driver constructed" << std::endl;

  rclcpp::shutdown();
  std::cout << "minimal_trossen_ros_crash: after shutdown" << std::endl;
  return 0;
}
