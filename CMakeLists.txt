cmake_minimum_required(VERSION 3.0.2)
project(system_id)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(system_id_node src/system_id_node.cpp)

add_dependencies(system_id_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(system_id_node
  ${catkin_LIBRARIES}
)