cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_trajectory_1 src/test_trajectory_1.cpp)
ament_target_dependencies(test_trajectory_1 rclcpp geometry_msgs nav2_msgs rclcpp_action)

add_executable(test_trajectory_2 src/test_trajectory_2.cpp)
ament_target_dependencies(test_trajectory_2 rclcpp geometry_msgs nav2_msgs rclcpp_action)

add_executable(test_trajectory_3 src/test_trajectory_3.cpp)
ament_target_dependencies(test_trajectory_3 rclcpp geometry_msgs nav2_msgs rclcpp_action)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

add_executable(kf src/kf.cpp)
ament_target_dependencies(kf rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs tf2 tf2_geometry_msgs Eigen3 message_filters)

add_executable(kf_wrong_quat src/kf_wrong_quat.cpp)
ament_target_dependencies(kf_wrong_quat rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs Eigen3 message_filters)

add_executable(ekf_odom src/ekf_odom.cpp)
ament_target_dependencies(ekf_odom rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3 message_filters)

add_executable(ekf src/ekf.cpp)
ament_target_dependencies(ekf rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3 message_filters)

add_executable(ekf_wrong_quat src/ekf_wrong_quat.cpp)
ament_target_dependencies(ekf_wrong_quat rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs sensor_msgs visualization_msgs Eigen3 message_filters)

add_executable(ekf_improved src/ekf_improved.cpp)
ament_target_dependencies(ekf_improved rclcpp std_msgs geometry_msgs nav2_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3 message_filters)

add_executable(pf src/pf.cpp)
ament_target_dependencies(pf rclcpp geometry_msgs sensor_msgs nav_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3)

add_executable(test_landmarks src/test_landmarks.cpp)
ament_target_dependencies(test_landmarks rclcpp geometry_msgs nav2_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3 message_filters)

install(TARGETS test_trajectory_1 test_trajectory_2 test_trajectory_3 kf ekf test_landmarks kf_wrong_quat ekf_odom ekf_wrong_quat ekf_improved pf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
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
