cmake_minimum_required(VERSION 3.0.2)
project(ur5e_mlp3240_task)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PRIVATE include)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
