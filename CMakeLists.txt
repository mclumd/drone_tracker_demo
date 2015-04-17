cmake_minimum_required(VERSION 2.8.3)
project(drone_tracker_demo)

find_package(PCL REQUIRED QUIET COMPONENTS common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros 
  pcl_conversions)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)


## Declare a cpp executable
#add_executable(${PROJECT_NAME}_node src/depth_tracker_node.cpp src/depth_tracker.cpp)
add_executable(${PROJECT_NAME}_node src/drone_tracker.cpp)


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARY_DIRS}
 )

#target_link_libraries(old_${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARY_DIRS}
# )



