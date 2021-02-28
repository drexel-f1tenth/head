cmake_minimum_required(VERSION 3.0.2)
project(pathfinder)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pathfinder
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

message("${CATKIN_PACKAGE_BIN_DESTINATION}")
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
