cmake_minimum_required(VERSION 3.8)
project(roboteq_motor_controller_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(roboteq_motor_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

generate_parameter_library(
  diff_odom_parameters # cmake target name for the parameter library
  src/diff_odom_parameters.yaml # path to input yaml file
)

generate_parameter_library(
  roboteq_motor_controller_driver_parameters # cmake target name for the parameter library
  src/roboteq_motor_controller_driver_parameters.yaml # path to input yaml file
)

add_executable(roboteq_motor_controller_driver
  src/roboteq_motor_controller_driver.cpp
)

target_include_directories(roboteq_motor_controller_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboteq_motor_controller_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(roboteq_motor_controller_driver
  rclcpp::rclcpp
  roboteq_motor_controller_driver_parameters
)

ament_target_dependencies(roboteq_motor_controller_driver
  rclcpp
  serial
  std_msgs
  geometry_msgs
  roboteq_motor_controller_msgs
)

add_executable(diff_odom
  src/diff_odom.cpp
)
target_include_directories(diff_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(diff_odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(diff_odom
  rclcpp::rclcpp
  diff_odom_parameters
)
ament_target_dependencies(diff_odom
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  roboteq_motor_controller_msgs
)

# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config 
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS roboteq_motor_controller_driver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS diff_odom
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
