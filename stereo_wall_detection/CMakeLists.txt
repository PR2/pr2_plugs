cmake_minimum_required(VERSION 2.8.3)
project(stereo_wall_detection)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros pcl_conversions pcl_msgs message_generation)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


#link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
set (ROS_BUILD_TYPE RelWithDebInfo)

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_service_files(
  FILES
  DetectWall.srv
)

generate_messages(
    DEPENDENCIES geometry_msgs
)

catkin_package(
    DEPENDS Eigen PCL 
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros pcl_msgs message_runtime geometry_msgs # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


add_executable (wall_extractor src/wall_extractor_pcl.cpp)
target_link_libraries(wall_extractor ${catkin_LIBRARIES})
add_dependencies(wall_extractor pcl_ros_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable (service_trigger src/service_trigger.cpp)
target_link_libraries(service_trigger ${catkin_LIBRARIES})
add_dependencies(service_trigger pcl_ros_generate_messages_cpp ${catkin_EXPORTED_TARGETS} stereo_wall_detection_gencpp)



install(FILES wall_extractor.launch CHANGELOG.rst DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



