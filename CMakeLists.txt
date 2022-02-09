cmake_minimum_required(VERSION 3.0.2)
project(jetson_camera_trigger)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    std_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jetson_camera_trigger src/jetson_camera_trigger_main.cpp src/jetson_camera_trigger_node.cpp src/jetson_gpio.cpp)
target_link_libraries(jetson_camera_trigger ${catkin_LIBRARIES})
add_dependencies(jetson_camera_trigger ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(
  TARGETS
    jetson_camera_trigger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install files
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)