cmake_minimum_required(VERSION 3.8)
project(drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sim_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create executable for cube position publishing
add_executable(position_pub src/position_publisher.cpp)
ament_target_dependencies(position_pub rclcpp gazebo_msgs geometry_msgs sim_msgs)

# Create executable for cube movement
add_executable(cube_mover src/cube_movement_setter.cpp)
ament_target_dependencies(cube_mover rclcpp gazebo_msgs geometry_msgs)

# Create executable for drone movement
add_executable(drone_mover src/drone_movement_setter.cpp)
ament_target_dependencies(drone_mover rclcpp gazebo_msgs geometry_msgs)

# Install the created executables
install(TARGETS
position_pub cube_mover drone_mover
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
