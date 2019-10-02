cmake_minimum_required(VERSION 2.8.3)
project(grasp_point_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  ur_kinematics
  rviz_visual_tools
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Boost REQUIRED)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_kinematics
#  CATKIN_DEPENDS roscpp ur_kinematics
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (tf2_listener src/tf2_with_pcd.cpp src/entropy.cpp  src/binsegmentation.cpp src/pointpose.cpp)
target_link_libraries(tf2_listener ${catkin_LIBRARIES})
target_link_libraries (tf2_listener ${PCL_LIBRARIES})
target_link_libraries(tf2_listener ${Boost_LIBRARIES})

add_executable (grasp_pose_pub src/grasp_pose_pub.cpp src/entropy.cpp  src/binsegmentation.cpp src/pointpose.cpp)
target_link_libraries(grasp_pose_pub ${catkin_LIBRARIES})
target_link_libraries (grasp_pose_pub ${PCL_LIBRARIES})
target_link_libraries(grasp_pose_pub ${Boost_LIBRARIES})

add_executable (transform_pub src/transform_pub.cpp)
target_link_libraries(transform_pub ${catkin_LIBRARIES})

add_executable (grasp_pose_transform src/grasp_pose_transform.cpp)
target_link_libraries(grasp_pose_transform ${catkin_LIBRARIES})