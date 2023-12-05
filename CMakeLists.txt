cmake_minimum_required(VERSION 3.5)
project(sensor_trigger)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig)
pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/sensor_trigger.cpp
  src/jetson_gpio.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GPIOD_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sensor_trigger::SensorTrigger"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
