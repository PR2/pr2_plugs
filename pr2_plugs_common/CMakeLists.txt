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
#rosbuild_add_library(${PROJECT_NAME} src/plugs_planner.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(pose_to_tf src/pose_to_tf.cpp)
rosbuild_add_executable(detector_stub src/detector_stub.cpp)
rosbuild_add_executable(detector_filter src/detector_filter.cpp)
rosbuild_add_executable(covariance_inserter src/covariance_inserter.cpp)

rosbuild_add_executable(joint_space_move_test EXCLUDE_FROM_ALL test/joint_space_move_test.cpp include/pr2_plugs_common/joint_space_move.h)
rosbuild_add_gtest_build_flags(joint_space_move_test)
rosbuild_add_rostest(test/joint_space_move_test.launch)
