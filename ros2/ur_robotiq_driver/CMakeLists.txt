cmake_minimum_required(VERSION 3.8)
project(ur_robotiq_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  PROGRAMS src/robotiq_hand_e_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
