cmake_minimum_required(VERSION 3.5)
project(pkg_13224067)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


# Install Python nodes (autonomous_bridge.py, multiplexer.py)
install(PROGRAMS
  pkg_13224067/autonomous_bridge.py
  pkg_13224067/multiplexer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
