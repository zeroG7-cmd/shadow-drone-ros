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
find_package(gazebo_msgs REQUIRED)
# Install package folders
# urdf/ deliberately not installed here - the old one was built for the
# spawn_entity.py pathway you're no longer using now that PX4's own SDF
# + MAVROS is the real flight path. Worth rebuilding properly once Stage
# 3 (camera bridge) actually needs TF, not before.
install(DIRECTORY
  launch
  worlds
  rviz
  config
  scripts
  nodes
  DESTINATION share/${PROJECT_NAME}
)

# Nodes launched via ros2 run need to be installed as executables (into
# lib/), not just copied as data files (into share/ above).
install(PROGRAMS
  nodes/mission_planner.py
  nodes/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
