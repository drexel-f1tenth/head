cmake_minimum_required(VERSION 3.0.2)
project(mcu_pub)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(ASAN_FLAGS -fno-omit-frame-pointer -fsanitize=address)
  target_compile_options(${PROJECT_NAME} PRIVATE ${ASAN_FLAGS})
  target_link_libraries(${PROJECT_NAME} ${ASAN_FLAGS})
endif()

message("${CATKIN_PACKAGE_BIN_DESTINATION}")
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
