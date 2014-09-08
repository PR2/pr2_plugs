cmake_minimum_required(VERSION 2.8.3)
project(checkerboard_pose_estimation)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport visual_pose_estimation outlet_pose_estimation)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/cvcalibinit_lowres.cpp src/detector.cpp src/estimator.cpp src/ros_detector.cpp)

catkin_package(
    DEPENDS roscpp opencv2 cv_bridge image_transport visual_pose_estimation outlet_pose_estimation
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES ${PROJECT_NAME}# TODO
)


install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

