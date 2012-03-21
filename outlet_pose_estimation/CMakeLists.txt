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
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(outlet_detection src/liboutlet/affine_transform.cpp src/liboutlet/constellation.cpp src/liboutlet/features.cpp src/liboutlet/geometric_hash.cpp src/liboutlet/generalized_hough.cpp src/liboutlet/gh_outlets.cpp src/liboutlet/learning.cpp src/liboutlet/one_way_descriptor_base.cpp src/liboutlet/one_way_descriptor.cpp src/liboutlet/one_way_outlets.cpp src/liboutlet/outlet_detector.cpp src/liboutlet/outlet_model.cpp src/liboutlet/outlet_tuple.cpp src/liboutlet/pca_features.cpp src/liboutlet/planar.cpp)
add_definitions(-D_OUTLET_HR -D_TUNING -D_USE_OUTLET_TUPLE -D_GHT -D_KDTREE -D_GHASH)
find_package(OpenCV REQUIRED)
target_link_libraries(outlet_detection ${OpenCV_LIBRARIES})

rosbuild_add_library(${PROJECT_NAME} src/detector.cpp src/estimator.cpp)
target_link_libraries(${PROJECT_NAME} outlet_detection)
