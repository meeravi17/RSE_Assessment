cmake_minimum_required(VERSION 3.0.2)
project(steer_bot_navigation)

# add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  sensor_msgs
  cv_bridge
  laser_geometry
  tf2_geometry_msgs
  tf2
  tf2_msgs
  message_generation
)

add_message_files(
  FILES
  
)

add_service_files(
  FILES
  OdometryResetRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES steer_bot_navigation laser_scan_pcl_lib
 DEPENDS system_lib opencv2 Eigen
 CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros tf2_geometry_msgs tf2 tf2_msgs message_runtime
 sensor_msgs
 cv_bridge
  
#  DEPENDS system_lib
)

include_directories(/usr/include/eigen3)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${Eigen_INCLUDE_DIRS}
  
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(laser_scan_pcl_lib  src/offline_localization_icp/laser_scan_pcl_lib.cpp)

add_library(steer_bot_remapper src/cmdvelocityRemapper.cpp)
target_link_libraries(steer_bot_remapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  
)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)


add_executable(offline_localization_icp src/offline_localization_icp/offline_localization_icp.cpp)
add_executable(steer_bot_odometry src/odometry_ackermann/odometry_ackermann.cpp)


target_link_libraries(offline_localization_icp laser_scan_pcl_lib ndt_omp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  yaml-cpp
  ndt_omp
)

target_link_libraries(laser_scan_pcl_lib ndt_omp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  yaml-cpp
  ndt_omp
)

target_link_libraries(steer_bot_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  
)

catkin_install_python(PROGRAMS src/lane_navigation/LaneDetection.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 catkin_install_python(PROGRAMS src/pf/particle_filter.py src/pf/match_estimator.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


install(TARGETS steer_bot_remapper steer_bot_odometry offline_localization_icp ndt_omp
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/pclomp
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
