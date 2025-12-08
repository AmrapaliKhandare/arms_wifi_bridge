cmake_minimum_required(VERSION 3.10.0)
project(arms_wifi_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libtrossen_arm REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
)

add_executable(leader_node
  src/leader_node_tcp.cpp
)

target_include_directories(leader_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/arms_wifi_bridge
)

ament_target_dependencies(leader_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Follower node (similar structure)
add_executable(follower_node
  src/follower_node_tcp.cpp
)

target_include_directories(follower_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/arms_wifi_bridge
)

ament_target_dependencies(follower_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install
install(TARGETS
  leader_node
  follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have them
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
#   OPTIONAL_DEPENDENCIES
# )

################# tests #############################
add_executable(test_plain_libtrossen test/plain_libtrossen_test.cpp)
target_link_libraries(test_plain_libtrossen
  libtrossen_arm
)
install(
  TARGETS test_plain_libtrossen
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#################
add_executable(test_empty_ros_node test/empty_ros_node.cpp)
ament_target_dependencies(test_empty_ros_node rclcpp)
target_link_libraries(test_empty_ros_node
  libtrossen_arm
)
install(
  TARGETS test_empty_ros_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#################
add_executable(minimal_trossen_ros_crash test/minimal_trossen_ros_crash.cpp)
ament_target_dependencies(minimal_trossen_ros_crash rclcpp)
target_link_libraries(minimal_trossen_ros_crash
  libtrossen_arm
)
install(
  TARGETS minimal_trossen_ros_crash
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
