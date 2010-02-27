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


rosbuild_add_executable(vision_detect_plug src/vision_detect_plug.cpp)

rosbuild_add_executable(vision_detect_outlet src/vision_detect_outlet.cpp)

#### Download outlet templates ####
rosbuild_download_data(http://pr.willowgarage.com/data/outlet_detection/outlet_templates_20091207.tar.gz outlet_templates_20091207.tar.gz 6ac4bfc16222132cdb17181761349c72)
# Unpack the tarball.
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/outlet_templates
                   COMMAND tar zxvf outlet_templates_20091207.tar.gz
                   COMMAND touch -c outlet_templates
                   DEPENDS ${PROJECT_SOURCE_DIR}/outlet_templates_20091207.tar.gz
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                   VERBATIM)
add_custom_target(download_and_unpack_data ALL
                  DEPENDS ${PROJECT_SOURCE_DIR}/outlet_templates)
# Note: Assuming knowledge of the specific download target here
add_dependencies(download_and_unpack_data download_data_outlet_templates_20091207.tar.gz)

################ Tests ###################
rosbuild_add_executable(fetch_and_stow_plug_test EXCLUDE_FROM_ALL test/action_server_tests/fetch_and_stow_plug_test.cpp)
rosbuild_add_gtest_build_flags(fetch_and_stow_plug_test)
rosbuild_add_rostest(test/action_server_tests/fetch_and_stow_plug_test.launch)

rosbuild_add_executable(detect_plug_test EXCLUDE_FROM_ALL test/action_server_tests/detect_plug_test.cpp)
rosbuild_add_gtest_build_flags(detect_plug_test)
rosbuild_add_rostest(test/action_server_tests/detect_plug_test.launch)

rosbuild_add_executable(detect_outlet_test EXCLUDE_FROM_ALL test/action_server_tests/detect_outlet_test.cpp)
rosbuild_add_gtest_build_flags(detect_outlet_test)
rosbuild_add_rostest(test/action_server_tests/detect_outlet_test.launch)

rosbuild_add_executable(detect_wall_norm_test EXCLUDE_FROM_ALL test/action_server_tests/detect_wall_norm_test.cpp)
rosbuild_add_gtest_build_flags(detect_wall_norm_test)
rosbuild_add_rostest(test/action_server_tests/detect_wall_norm_test.launch)

rosbuild_add_executable(wiggle_plug_test EXCLUDE_FROM_ALL test/action_server_tests/wiggle_plug_test.cpp)
rosbuild_add_gtest_build_flags(wiggle_plug_test)
rosbuild_add_rostest(test/action_server_tests/wiggle_plug_test.launch)

rosbuild_add_executable(vision_detect_outlet_and_plug_test EXCLUDE_FROM_ALL test/action_server_tests/vision_detect_outlet_and_plug_test.cpp)
rosbuild_add_gtest_build_flags(vision_detect_outlet_and_plug_test)
rosbuild_add_rostest(test/action_server_tests/vision_detect_outlet_and_plug_test.launch)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
