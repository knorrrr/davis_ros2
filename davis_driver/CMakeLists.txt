cmake_minimum_required(VERSION 3.8)
project(davis_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(LIBCAER_HAVE_OPENCV 1)
SET(LIBCAER_FRAMECPP_OPENCV_INSTALLED true)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(libcaer REQUIRED)
find_package(ev_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
add_executable(davis_pub src/driver.cpp include/davis_driver/davis_driver.hpp)
target_include_directories(davis_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(davis_pub
  libcaer::caer
)
ament_target_dependencies(davis_pub
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  libcaer
  ev_msgs
)
install(TARGETS
  davis_pub
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
