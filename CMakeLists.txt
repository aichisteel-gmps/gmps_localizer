cmake_minimum_required(VERSION 3.8)
project(gmps_localizer_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gmps_msgs_package REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(gmps_localizer
 src/gmps_localizer.cpp
)

ament_target_dependencies(gmps_localizer
 rclcpp
 std_msgs
 gmps_msgs_package
 geometry_msgs
 tf2_ros
 tf2) 

install(TARGETS
  gmps_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  DATA
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
