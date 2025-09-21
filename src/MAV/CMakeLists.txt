cmake_minimum_required(VERSION 3.8)
project(mav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(
  include
  ${CURSES_INCLUDE_DIR}
)

add_executable(mav1 node/mav1.cpp)
ament_target_dependencies(mav1 rclcpp std_msgs geometry_msgs px4_msgs tf2)

add_executable(mav2 node/mav2.cpp)
ament_target_dependencies(mav2 rclcpp std_msgs geometry_msgs px4_msgs tf2)
add_executable(mav3 node/mav3.cpp)
ament_target_dependencies(mav3 rclcpp std_msgs geometry_msgs px4_msgs tf2)
add_executable(mav4 node/mav4.cpp)
ament_target_dependencies(mav4 rclcpp std_msgs geometry_msgs px4_msgs tf2)

install(TARGETS
  mav1
  mav2
  mav3
  mav4
  DESTINATION lib/${PROJECT_NAME})


  # Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
