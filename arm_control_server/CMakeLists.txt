# CMakeLists.txt for ARM CONTROL SERVER (Pure C++, NO ROS)
# Build this separately from the ROS workspace

cmake_minimum_required(VERSION 3.8)
project(arm_control_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find libtrossen_arm (adjust path if needed)
find_package(libtrossen_arm REQUIRED)

# ARM Control Server executable
add_executable(arm_control_server
  src/arm_control_server.cpp
)

target_include_directories(arm_control_server PRIVATE
  include
)

target_link_libraries(arm_control_server
  libtrossen_arm
)

# Install
install(TARGETS arm_control_server
  DESTINATION bin
)