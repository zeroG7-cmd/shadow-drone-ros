cmake_minimum_required(VERSION 3.5)
project(shadow_drone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)


# Install Python launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF/XACRO files
install(
  DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
