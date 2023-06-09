cmake_minimum_required(VERSION 3.8)
project(grasp_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(grasp_moveit src/grasp_moveit.cpp)
ament_target_dependencies(
  grasp_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

target_compile_features(grasp_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(pose_publisher src/pose_publisher.cpp)
ament_target_dependencies(
  pose_publisher
  "moveit_ros_planning_interface"
  "rclcpp"
)
target_compile_features(pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



install(TARGETS 
  grasp_moveit 
  pose_publisher
  DESTINATION lib/${PROJECT_NAME})

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
