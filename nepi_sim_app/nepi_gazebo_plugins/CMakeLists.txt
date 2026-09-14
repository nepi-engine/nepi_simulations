cmake_minimum_required(VERSION 3.0.2)
project(nepi_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_dev
  gazebo_ros
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(nepi_crab_steer_plugin SHARED src/crab_steer_plugin.cpp)
target_link_libraries(nepi_crab_steer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS nepi_crab_steer_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
