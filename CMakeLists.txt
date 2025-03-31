cmake_minimum_required(VERSION 3.8)  
project(ros2_action_mux)  

# Set the default C++ standard to C++17 if not specified  
if(NOT CMAKE_CXX_STANDARD)  
  set(CMAKE_CXX_STANDARD 17)  
endif()  

# Compiler settings for GNU or Clang  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  
  add_compile_options(-Wall -Wextra -Wpedantic)  # Enable warnings  
endif()  

# Find required packages  
find_package(ament_cmake REQUIRED)  
find_package(rclcpp REQUIRED)  
find_package(rclcpp_action REQUIRED)  
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)  
find_package(action_msgs REQUIRED)  
find_package(rosidl_default_generators REQUIRED)  

# Generate action interfaces  
rosidl_generate_interfaces(${PROJECT_NAME}  
  "action/ProcessRequest.action"  
  DEPENDENCIES std_msgs  # Specify dependencies for the action interface  
)  

# Create a symlink for including the generated header files  
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")  

# Include directories for headers  
include_directories(include)  

# Action Server executable  
add_executable(action_server src/action_server.cpp)  
target_link_libraries(action_server ${cpp_typesupport_target})  # Link typesupport  
ament_target_dependencies(action_server  
  rclcpp  
  rclcpp_action  
  std_msgs  
)  

# Action Client executable  
add_executable(action_client src/action_client.cpp)  
target_link_libraries(action_client ${cpp_typesupport_target})  # Link typesupport  
ament_target_dependencies(action_client  
  rclcpp  
  rclcpp_action  
  std_msgs  
)  

# Generic Subscriber executable  
add_executable(generic_subscriber src/generic_subscriber.cpp)  
ament_target_dependencies(generic_subscriber  
  rclcpp  
  std_msgs  
  sensor_msgs  
  geometry_msgs  
)  

# Test Publishers  
add_executable(string_publisher src/test_publishers/string_publisher.cpp)  
ament_target_dependencies(string_publisher  
  rclcpp  
  std_msgs  
)  

add_executable(int_publisher src/test_publishers/int_publisher.cpp)  
ament_target_dependencies(int_publisher  
  rclcpp  
  std_msgs  
)  

add_executable(pose_publisher src/test_publishers/pose_publisher.cpp)  
ament_target_dependencies(pose_publisher  
  rclcpp  
  geometry_msgs  
)  

# Install targets (executables)  
install(TARGETS  
  action_server  
  action_client  
  generic_subscriber  
  string_publisher  
  int_publisher  
  pose_publisher  
  DESTINATION lib/${PROJECT_NAME}  # Specify installation destination  
)  

# Install launch files  
install(DIRECTORY  
  launch  
  DESTINATION share/${PROJECT_NAME}  
)  

# Install include directory for header files  
install(DIRECTORY include/  
  DESTINATION include  
)  

# Call ament package to finalize the build configuration  
ament_package()  

using namespace std;  # Use std namespace for convenience, though not common in CMake files  