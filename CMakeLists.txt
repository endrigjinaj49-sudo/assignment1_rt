cmake_minimum_required(VERSION 3.8)
project(assignment1_rt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# ---------- C++ executables ----------

add_executable(ui src/ui.cpp)
ament_target_dependencies(ui rclcpp geometry_msgs)

add_executable(distance src/distance.cpp)
ament_target_dependencies(distance rclcpp geometry_msgs std_msgs turtlesim)

install(TARGETS
  ui
  distance
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Python script (spawner) ----------

install(PROGRAMS
  scripts/turtle_spawn.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
