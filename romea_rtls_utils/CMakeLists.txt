cmake_minimum_required(VERSION 3.5)
project(romea_rtls_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_rtls REQUIRED)
find_package(romea_core_rtls_transceiver REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_rtls_transceiver_utils REQUIRED)
find_package(romea_rtls_transceiver_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rtls_parameters.cpp
  src/rtls_communication_hub.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  romea_core_common
  romea_core_rtls
  romea_common_utils
  romea_rtls_transceiver_utils
  romea_rtls_transceiver_msgs
  nav_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_dependencies(rclcpp)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_rtls)
ament_export_dependencies(romea_core_rtls_transceiver)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(romea_rtls_transceiver_utils)
ament_export_dependencies(romea_rtls_transceiver_msgs)
ament_export_dependencies(nav_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()


