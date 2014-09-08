cmake_minimum_required(VERSION 2.8.3)
project(stereo_wall_detection)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros pcl_conversions pcl_msgs)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
set (ROS_BUILD_TYPE RelWithDebInfo)

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_service_files(
  FILES
  DetectWall.srv
)

add_executable (wall_extractor src/wall_extractor_pcl.cpp)
target_link_libraries(wall_extractor ${catkin_LIBRARIES})
add_dependencies(wall_extractor pcl_ros_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable (service_trigger src/service_trigger.cpp)
target_link_libraries(service_trigger ${catkin_LIBRARIES})
add_dependencies(service_trigger pcl_ros_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
    DEPENDS Boost Eigen PCL roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(FILES wall_extractor.launch CHANGELOG.rst DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



