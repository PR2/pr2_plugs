# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(stereo_wall_detection)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required (VERSION 2.4.6)
include_directories(include)

# CATKIN_MIGRATION: removed during catkin migration
# include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
#set (ROS_BUILD_TYPE Debug)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init ()

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories ()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_service_files(
  FILES
  # TODO: List your msg files here
  DetectWall.srv
)


add_executable (wall_extractor src/wall_extractor_pcl.cpp)
target_link_libraries(wall_extractor ${catkin_LIBRARIES})

add_executable (service_trigger src/service_trigger.cpp)
target_link_libraries(service_trigger ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(wall_extractor ${Boost_LIBRARIES})
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
    DEPENDENCIES std_msgs geometry_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp geometry_msgs sensor_msgs angles visualization_msgs pcl_ros
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

