cmake_minimum_required(VERSION 3.5)
project(shadow_drone)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)
# gazebo_ros / gazebo_msgs find_package() calls removed here (they were
# previously kept as a formality, even though nothing in this
# CMakeLists.txt actually compiles against either - this package is
# Python-only). Removed for a concrete reason, not just tidiness: on
# this machine, installing ros-humble-gazebo-ros-pkgs to satisfy them
# pulls in the real "gazebo" (Classic 11) metapackage, which Conflicts
# at the apt level with gz-tools2 - already installed as part of the
# working Gazebo Harmonic setup Gate 1/Gate 2 actually fly on. There is
# no way to have both installed at once on this box. Since nothing here
# links against gazebo_ros/gazebo_msgs anyway, the right fix is to stop
# requiring them, not to fight that conflict. The one real user of
# gazebo_ros, launch/simulation.launch.py's spawn_entity.py node, is
# already flagged elsewhere as the old PX4/Classic pathway nothing
# currently runs through (Gate 1/Gate 2 flew via manual sim_vehicle.py
# / gz sim commands instead) and due for its own ArduPilot+Harmonic
# rewrite - not reintroduced here.

# shadow/ is the RPG-layered package (body/senses/skills/mind) - a
# pure-Python subpackage installed alongside this ament_cmake package,
# not a standalone script, so it gets ament_python_install_package
# instead of the install(PROGRAMS ...) treatment below.
ament_python_install_package(shadow)

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
  nodes/run_skill.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
