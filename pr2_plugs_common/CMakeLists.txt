# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.8.3)
project(pr2_plugs_common)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs geometry_msgs actionlib pr2_controllers_msgs std_srvs orocos_kdl tf2_geometry_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/plugs_planner.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_dependencies(${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(pose_to_tf src/pose_to_tf.cpp)
target_link_libraries(pose_to_tf ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(detector_stub src/detector_stub.cpp)
target_link_libraries(detector_stub ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(detector_filter src/detector_filter.cpp)
target_link_libraries(detector_filter ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


add_executable(covariance_inserter src/covariance_inserter.cpp)
target_link_libraries(covariance_inserter ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(joint_space_move_test EXCLUDE_FROM_ALL test/joint_space_move_test.cpp include/pr2_plugs_common/joint_space_move.h)

# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  rosbuild_add_gtest_build_flags(joint_space_move_test)
#add_rostest(test/joint_space_move_test.launch)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf bfl std_msgs geometry_msgs actionlib pr2_controllers_msgs std_srvs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include include/${PROJECT_NAME}# TODO include
    LIBRARIES # TODO
)

## Generate added messages and services with any dependencies listed here
