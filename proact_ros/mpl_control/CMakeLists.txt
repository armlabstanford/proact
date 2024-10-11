cmake_minimum_required(VERSION 3.1.3)
project(mpl_control)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    roscpp
    rospy
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    gazebo_grasp_plugin
    gazebo_grasp_plugin_ros
    std_msgs
    topic_utils
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES grasp.action 
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs   std_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    gazebo_grasp_plugin
    gazebo_grasp_plugin_ros
    std_msgs
    actionlib 
    actionlib_msgs 
    message_generation 
    roscpp
    rospy 
  DEPENDS
    EIGEN3
    GAZEBO
)


install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mpl_controlConfig.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mpl_control"
)


include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${gazebo_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${gazebo_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_server src/grasp_server.cpp)
add_executable(grasp_client src/grasp_client.cpp)

add_dependencies(grasp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(grasp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  grasp_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  grasp_client
  ${catkin_LIBRARIES}
)