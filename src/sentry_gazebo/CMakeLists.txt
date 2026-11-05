cmake_minimum_required(VERSION 3.16)
project(sentry_gazebo)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_map.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentry_bringup/config/tactics.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/maps
  RESULT_VARIABLE map_result
)
if(NOT map_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate the reference occupancy map")
endif()
install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/maps DESTINATION share/${PROJECT_NAME})
ament_package()
