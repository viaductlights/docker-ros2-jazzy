cmake_minimum_required(VERSION 3.8)
project(test_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(rclcpp_components REQUIRED)

#add_library(test_trajectory_component SHARED
#	src/path.cpp)

#target_include_directories(test_trajectory_component  PRIVATE
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<INSTALL_INTERFACE:include>)

#rclcpp_components_register_node(test_trajectory_component PLUGIN "test_trajectory::TestPath" EXECUTABLE path)

#target_compile_definitions(test_trajectory_component
#	PRIVATE "TEST_TRAJECTORY_BUILDING_DLL")

add_executable(path src/path.cpp)
ament_target_dependencies(path rclcpp geometry_msgs nav2_msgs rclcpp_action)

#rclcpp_components_register_node(test_trajectory_component
#	PLUGIN "test_trajectory::TestPath"
#	EXECUTABLE path)

install(TARGETS
	#test_trajectory_component
  path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
  #  rclcpp_components
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
