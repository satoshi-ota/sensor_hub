cmake_minimum_required(VERSION 2.8.3)
project(sensor_hub)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  hongo_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
   cfg/SensorHub.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_hub
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs hongo_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sensor_hub
    src/library/sensor_hub.cpp
)

add_executable(sensor_hub_node src/nodes/sensor_hub_node.cpp)
add_dependencies(sensor_hub_node sensor_hub_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_hub_node sensor_hub ${catkin_LIBRARIES})
