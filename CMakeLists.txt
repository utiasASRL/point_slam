cmake_minimum_required(VERSION 3.0.2)
project(point_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions 
  message_filters 
  eigen_conversions
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  DEPENDS pcl_ros roscpp rospy std_msgs rosbag sensor_msgs pcl_conversions nav_msgs geometry_msgs 
  move_base_msgs actionlib tf tf_conversions message_filters eigen_conversions
	INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a C++ library

add_library(npm_ply_in src/npm_ply/ply_file_in.cc)
target_link_libraries(npm_ply_in ${catkin_LIBRARIES})

add_library(npm_ply_out src/npm_ply/ply_file_out.cc)
target_link_libraries(npm_ply_out ${catkin_LIBRARIES})

add_library(cloud src/cloud/cloud.cpp)
target_link_libraries(cloud npm_ply_in npm_ply_out ${catkin_LIBRARIES})

add_library(grid_subsampling src/grid_subsampling/grid_subsampling.cpp)
target_link_libraries(grid_subsampling cloud ${catkin_LIBRARIES})

add_library(polar_processing src/polar_processing/polar_processing.cpp)
target_link_libraries(polar_processing cloud ${catkin_LIBRARIES})

add_library(pointmap src/pointmap/pointmap.cpp)
target_link_libraries(pointmap cloud ${catkin_LIBRARIES})

add_library(icp src/icp/icp.cpp)
target_link_libraries(icp pointmap cloud ${catkin_LIBRARIES})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(frame_update src/frame_update.cpp)
target_link_libraries(frame_update icp pointmap polar_processing grid_subsampling cloud ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_point_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
