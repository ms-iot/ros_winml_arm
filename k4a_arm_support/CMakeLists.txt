cmake_minimum_required(VERSION 2.8.3)
project(k4a_arm_support)

add_compile_options(/Od)
add_compile_options(/std:c++latest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  Eigen3 
  geometry_msgs
  interactive_markers
  visualization_msgs
  winml_msgs
  marker_msgs
  moveit_ros_planning_interface
)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})
catkin_package(
  LIBRARIES k4a_arm_support
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp roslib std_msgs visualization_msgs moveit_ros_planning_interface winml_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/k4a_arm_support.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch meshes urdf calibration viz log
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
