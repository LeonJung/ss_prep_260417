cmake_minimum_required(VERSION 3.8)
project(dg5f_hand_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dg5f_sim_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
