cmake_minimum_required(VERSION 2.8.3)
project(sensor_hub)
add_compile_options(-std=c++11)


set(PACKAGE_DEPENDENCIES
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

generate_dynamic_reconfigure_options(
    cfg/SensorHub.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/library/sensor_hub.cpp
    src/library/protocol.cpp
)

add_executable(sensor_hub_node src/nodes/sensor_hub_node.cpp)
add_dependencies(sensor_hub_node sensor_hub_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_hub_node sensor_hub ${catkin_LIBRARIES})
