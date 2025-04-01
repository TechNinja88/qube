cmake_minimum_required(VERSION 3.8)
project(qube_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include(FetchContent)
set(SERIAL_BUILD_EXAMPLES OFF)
FetchContent_Declare(
  serial
  GIT_REPOSITORY https://github.com/markaren/serial.git
  GIT_TAG main
  )
  FetchContent_MakeAvailable(serial)
    
  add_library(qube_driver
  SHARED
  src/qube_driver.cpp
  )
  
  target_include_directories(qube_driver
  PRIVATE
  include
  )
  
  target_link_libraries(qube_driver serial)
  ament_target_dependencies(qube_driver
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  )
  
  target_link_libraries(qube_driver serial)
  pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)
  
  
  ## EXPORTS
  ament_export_dependencies(
    hardware_interface
    controller_manager
    pluginlib
    serial
    rclcpp
    )
    ament_export_libraries(
      qube_driver
      )
      
install(
  TARGETS qube_driver
  DESTINATION lib
)

# Install launch files.
install(DIRECTORY
  config
  launch
  ros2_control
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
