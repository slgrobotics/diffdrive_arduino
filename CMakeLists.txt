cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(diffdrive_arduino SHARED
  src/diffdrive_arduino.cpp src/wheel.cpp src/config.cpp src/arduino_comms.cpp)
add_library(fake_robot SHARED src/fake_robot.cpp src/wheel.cpp src/config.cpp)
foreach(target diffdrive_arduino fake_robot)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${target} PUBLIC
    hardware_interface::hardware_interface rclcpp::rclcpp pluginlib::pluginlib)
endforeach()
target_include_directories(diffdrive_arduino SYSTEM PUBLIC ${serial_INCLUDE_DIRS})
target_link_libraries(diffdrive_arduino PUBLIC ${serial_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)
pluginlib_export_plugin_description_file(hardware_interface fake_robot_hardware.xml)

install(TARGETS diffdrive_arduino fake_robot
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY controllers description DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(diffdrive_arduino fake_robot)
ament_export_dependencies(hardware_interface serial rclcpp pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel_config test/test_wheel_config.cpp)
  target_link_libraries(test_wheel_config fake_robot)
  if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    ament_add_gtest(test_hardware test/test_hardware.cpp)
    target_link_libraries(test_hardware diffdrive_arduino fake_robot util Threads::Threads)
  endif()
endif()

ament_package()
