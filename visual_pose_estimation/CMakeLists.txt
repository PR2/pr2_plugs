cmake_minimum_required(VERSION 2.8.3)
project(visual_pose_estimation)
find_package(catkin REQUIRED COMPONENTS roscpp image_geometry tf )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


catkin_package(
    DEPENDS roscpp opencv2 image_geometry tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES ${PROJECT_NAME} # TODO
)

add_library(${PROJECT_NAME} src/pose_estimator.cpp src/planar.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME} 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
