cmake_minimum_required(VERSION 3.12)
project(sam_bot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts (run via `ros2 run sam_bot_bringup <name>.py`).
install(PROGRAMS
  scripts/square_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
