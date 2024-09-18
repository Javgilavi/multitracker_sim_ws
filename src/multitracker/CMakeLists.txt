cmake_minimum_required(VERSION 3.8)
project(multitracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${EIGEN3_INCLUDE_DIR} include)

# Create executable for tracker node
add_executable(tracker src/tracker.cpp)
ament_target_dependencies(tracker rclcpp geometry_msgs sim_msgs visualization_msgs Eigen3)

# Install the created executables
install(TARGETS
  tracker
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config file
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
