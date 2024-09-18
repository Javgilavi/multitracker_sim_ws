cmake_minimum_required(VERSION 3.8)
project(sim_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the msg files
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Dimension.msg"
"msg/Adsb.msg"
DEPENDENCIES geometry_msgs)

ament_package()