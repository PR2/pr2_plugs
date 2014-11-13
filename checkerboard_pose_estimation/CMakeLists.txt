cmake_minimum_required(VERSION 2.8.3)
project(checkerboard_pose_estimation)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport outlet_pose_estimation visual_pose_estimation)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp cv_bridge image_transport visual_pose_estimation outlet_pose_estimation
    CATKIN_DEPENDS visual_pose_estimation# TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES ${PROJECT_NAME}# TODO
)


add_library(${PROJECT_NAME} src/cvcalibinit_lowres.cpp src/detector.cpp src/estimator.cpp src/ros_detector.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBARIES} ${visual_pose_estimation_LIBRARIES})
add_dependencies(${PROJECT_NAME} visual_pose_estimation_EXPORTED_TARGETS visual_pose_estimation_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME} 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
