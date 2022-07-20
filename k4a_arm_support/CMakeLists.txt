cmake_minimum_required(VERSION 2.8.3)
project(k4a_arm_support)

add_compile_options(/Od)
add_compile_options(/std:c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  interactive_markers
  visualization_msgs
  ros_msft_onnx_msgs
  moveit_ros_planning_interface
)
catkin_package(
  LIBRARIES k4a_arm_support
  CATKIN_DEPENDS 
    geometry_msgs 
    interactive_markers 
    roscpp 
    roslib 
    std_msgs 
    visualization_msgs 
    moveit_ros_planning_interface 
    ros_msft_onnx_msgs
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
