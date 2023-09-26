cmake_minimum_required(VERSION 3.5)
project(romea_rtls_data_broadcaster)

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
  src/rtls_data_broadcaster.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ROMEA_RTLS_COORDINATORS_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  romea_core_common
  romea_core_rtls  
  romea_core_rtls_transceiver
  romea_common_utils
  romea_rtls_transceiver_utils
  romea_rtls_transceiver_msgs
  nav_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::RTLSPose2DBroadcaster"
  EXECUTABLE rtls_pose2d_broadcaster_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::RTLSTwist2DBroadcaster"
  EXECUTABLE rtls_twist2d_broadcaster_node
)


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
#   ament_lint_auto_find_test_dependencies()
#   # add_subdirectory(test)
# endif()

ament_package()


