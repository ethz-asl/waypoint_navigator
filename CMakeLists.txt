cmake_minimum_required(VERSION 2.8.3)
project(waypoint_navigator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(geodetic_utils REQUIRED)

catkin_simple()

cs_add_executable(waypoint_navigator_node src/waypoint_navigator_node.cpp)

add_dependencies(waypoint_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(waypoint_navigator_node ${catkin_LIBRARIES})

cs_install()
cs_export()
