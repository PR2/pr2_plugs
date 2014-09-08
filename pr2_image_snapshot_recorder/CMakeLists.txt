cmake_minimum_required(VERSION 2.8.3)
project(pr2_image_snapshot_recorder)
find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs sensor_msgs rosbag std_msgs)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_action_files(
  FILES
  ImageSnapshot.action
)
include_directories(${Boost_INCLUDE_DIRS})
generate_messages(
   DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
    DEPENDS rospy actionlib actionlib_msgs sensor_msgs rosbag
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/image_snapshotter.py scripts/send_trigger_goal.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


