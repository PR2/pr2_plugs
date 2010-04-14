cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_library(${PROJECT_NAME} src/trajectory_generation.cpp)

rosbuild_add_executable(joint_trajectory_generator src/joint_trajectory_generator.cpp include/joint_trajectory_generator/trajectory_unwrap.h)


rosbuild_add_executable(trajectory_generation_test test/trajectory_generation_test.cpp src/trajectory_generation.cpp)

rosbuild_add_executable(trajectory_unwrap_test EXCLUDE_FROM_ALL test/trajectory_unwrap_test.cpp include/joint_trajectory_generator/trajectory_unwrap.h)
rosbuild_add_gtest_build_flags(trajectory_unwrap_test)
rosbuild_add_rostest(test/trajectory_unwrap_test.launch)
