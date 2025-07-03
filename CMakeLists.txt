cmake_minimum_required(VERSION 3.8)
project(elevation_platform)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(elevation_platform_msgs REQUIRED)

add_executable(elevation_platform_node 
  src/elevation_platform_node.cpp
)
target_include_directories(elevation_platform_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(elevation_platform_node
  rclcpp
  can_msgs

  elevation_platform_msgs
)
target_compile_features(elevation_platform_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(joint_motor_driver_node 
  src/joint_motor_driver_node.cpp
)
target_include_directories(joint_motor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_motor_driver_node
  rclcpp
  std_msgs
  std_srvs
  can_msgs

  elevation_platform_msgs
)
target_compile_features(joint_motor_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  elevation_platform_node
  joint_motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
