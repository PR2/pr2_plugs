# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_plugs_actions)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS actionlib roscpp rospy actionlib_msgs geometry_msgs pr2_controllers_msgs pr2_controller_manager sensor_msgs tf tf_conversions tf2_ros tf2_geometry_msgs pr2_msgs std_srvs move_base_msgs dynamic_reconfigure pr2_common_action_msgs pr2_plugs_common pr2_plugs_msgs stereo_wall_detection checkerboard_pose_estimation outlet_pose_estimation image_transport joint_trajectory_action_tools pr2_arm_move_ik costmap_2d base_local_planner pr2_image_snapshot_recorder smach smach_ros cv_bridge)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
#  FILES
  # TODO: List your msg files here
#)


#uncomment if you have defined services
#add_service_files(
#  FILES
#  # TODO: List your msg files here
#)
find_package(OpenCV REQUIRED)

add_executable(vision_detect_plug src/vision_detect_plug.cpp)
target_link_libraries(vision_detect_plug ${OpenCV_LIBRARIES})
add_executable(vision_detect_outlet src/vision_detect_outlet.cpp)
target_link_libraries(vision_detect_outlet ${OpenCV_LIBRARIES})

add_executable(align_base src/align_base.cpp)
add_executable(move_base_omnidirectional src/move_base_omnidirectional.cpp)

#### Download outlet templates ####
download_data(http://pr.willowgarage.com/data/outlet_detection/outlet_templates_20091207.tar.gz build/outlet_templates_20091207.tar.gz 6ac4bfc16222132cdb17181761349c72)
# Unpack the tarball.

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/outlet_templates
                   COMMAND tar zxvf build/outlet_templates_20091207.tar.gz
                   COMMAND touch -c outlet_templates
                   DEPENDS ${PROJECT_SOURCE_DIR}/build/outlet_templates_20091207.tar.gz
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                   VERBATIM)
add_custom_target(download_and_unpack_data ALL
                  DEPENDS ${PROJECT_SOURCE_DIR}/outlet_templates)
# Note: Assuming knowledge of the specific download target here
add_dependencies(download_and_unpack_data download_data_outlet_templates_20091207.tar.gz)

################ Tests ###################
add_executable(fetch_and_stow_plug_test EXCLUDE_FROM_ALL test/action_server_tests/fetch_and_stow_plug_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(fetch_and_stow_plug_test)
add_rostest(test/action_server_tests/fetch_and_stow_plug_test.launch)
target_link_libraries(fetch_and_stow_plug_test ${OpenCV_LIBS})

add_executable(detect_plug_test EXCLUDE_FROM_ALL test/action_server_tests/detect_plug_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(detect_plug_test)
add_rostest(test/action_server_tests/detect_plug_test.launch)
target_link_libraries(detect_plug_test ${OpenCV_LIBS})

add_executable(detect_wall_norm_test EXCLUDE_FROM_ALL test/action_server_tests/detect_wall_norm_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(detect_wall_norm_test)
add_rostest(test/action_server_tests/detect_wall_norm_test.launch)
target_link_libraries(detect_wall_norm_test ${OpenCV_LIBS})

add_executable(wiggle_plug_test EXCLUDE_FROM_ALL test/action_server_tests/wiggle_plug_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(wiggle_plug_test)
add_rostest(test/action_server_tests/wiggle_plug_test.launch)
target_link_libraries(wiggle_plug_test ${OpenCV_LIBS})

#add_executable(vision_detect_outlet_and_plug_test EXCLUDE_FROM_ALL test/action_server_tests/vision_detect_outlet_and_plug_test.cpp)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(vision_detect_outlet_and_plug_test)
#add_rostest(test/action_server_tests/vision_detect_outlet_and_plug_test.launch)


#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS actionlib roscpp rospy actionlib_msgs geometry_msgs pr2_controllers_msgs pr2_controller_manager sensor_msgs tf kdl tf_conversions tf2_ros tf2_geometry_msgs pr2_msgs std_srvs move_base_msgs dynamic_reconfigure pr2_common_action_msgs pr2_plugs_common pr2_plugs_msgs stereo_wall_detection checkerboard_pose_estimation outlet_pose_estimation image_transport joint_trajectory_action_tools pr2_arm_move_ik costmap_2d base_local_planner pr2_image_snapshot_recorder smach smach_ros cv_bridge opencv2
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

