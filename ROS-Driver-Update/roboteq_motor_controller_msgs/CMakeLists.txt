cmake_minimum_required(VERSION 3.8)
project(roboteq_motor_controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Query.msg"
  DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

# add_executable(roboteq_motor_controller_driver_node
#   src/roboteq_motor_controller_driver_node.cpp
# )

# target_include_directories(roboteq_motor_controller_driver_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(roboteq_motor_controller_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies(roboteq_motor_controller_driver_node
#   rclcpp
# )

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
