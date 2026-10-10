cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(diffdrive_arduino SHARED
  src/diffdrive_arduino.cpp
  src/wheel.cpp
  src/range.cpp
  src/battery.cpp
  src/arduino_comms.cpp
)

target_include_directories(diffdrive_arduino
  PRIVATE
    include
    ${serial_INCLUDE_DIRS}
)

target_link_libraries(diffdrive_arduino PRIVATE
  ${hardware_interface_TARGETS}
  ${controller_manager_TARGETS}
  ${serial_LIBRARIES}
  rclcpp::rclcpp
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  robot_hardware.xml
)

install(
  TARGETS diffdrive_arduino
  DESTINATION lib
)

install(
  DIRECTORY
    controllers
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(diffdrive_arduino)

ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

if(BUILD_TESTING)
  enable_testing()
  find_package(diff_drive_controller REQUIRED)
  add_executable(encoder_regression_test tests/encoder_regression_test.cpp src/wheel.cpp)
  target_include_directories(encoder_regression_test PRIVATE include)
  target_link_libraries(encoder_regression_test PRIVATE diff_drive_controller::diff_drive_controller)
  add_test(NAME encoder_regression_test COMMAND encoder_regression_test)
endif()

ament_package()
