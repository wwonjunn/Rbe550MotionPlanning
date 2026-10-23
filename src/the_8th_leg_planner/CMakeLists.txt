cmake_minimum_required(VERSION 3.20)
project(the_8th_leg_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(rrt_planner src/rrt_planner.cpp)
target_include_directories(rrt_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rrt_planner PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_link_libraries(
rrt_planner
${rclcpp_TARGETS}
${moveit_core_TARGETS}
${moveit_ros_planning_TARGETS}
${visualization_msgs_TARGETS}
)

add_executable(rrt_planner_star src/rrt_planner_star.cpp)
target_include_directories(rrt_planner_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rrt_planner_star PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_link_libraries(
rrt_planner_star
${rclcpp_TARGETS}
${moveit_core_TARGETS}
${moveit_ros_planning_TARGETS}
${visualization_msgs_TARGETS}
)

install(TARGETS rrt_planner rrt_planner_star
DESTINATION lib/${PROJECT_NAME})

install(TARGETS rrt_planner
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
