cmake_minimum_required(VERSION 3.10)
project(omnibot_firmware)

# ---------------------------------------------------------------------------
#  Compiler setup
# ---------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
#  Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)

# Use pkg-config to locate LibSerial
pkg_check_modules(SERIAL libserial REQUIRED)

# ---------------------------------------------------------------------------
#  Library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/omnibot_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LDFLAGS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# ---------------------------------------------------------------------------
#  Pluginlib export (ROS 2 Jazzy style)
# ---------------------------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface omnibot_interface.xml)

# ---------------------------------------------------------------------------
#  Installation
# ---------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---------------------------------------------------------------------------
#  Export for downstream packages
# ---------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
