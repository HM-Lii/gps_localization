cmake_minimum_required(VERSION 3.0.2)
project(gps_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
)
add_subdirectory(third_party/GeographicLib)
find_package(PahoMqttCpp REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES your_package_name
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
)

include_directories(
  include
  "${PROJECT_SOURCE_DIR}/include"
  ${catkin_INCLUDE_DIRS}
  third_party/GeographicLib/include
)

add_executable(gps_localization src/gps_localization.cpp)
target_link_libraries(gps_localization ${catkin_LIBRARIES} ${tf_LIBRARIES} libGeographiccc PahoMqttCpp::paho-mqttpp3)
