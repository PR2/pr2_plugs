cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
#set (ROS_BUILD_TYPE Debug)
rosbuild_init ()

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

rosbuild_add_boost_directories ()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_gensrv ()

rosbuild_add_executable (wall_extractor src/wall_extractor_pcl.cpp)
rosbuild_add_executable (service_trigger src/service_trigger.cpp)
rosbuild_link_boost(wall_extractor thread)
