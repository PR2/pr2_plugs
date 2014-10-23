cmake_minimum_required(VERSION 2.8.3)
project(outlet_pose_estimation)
find_package(catkin REQUIRED COMPONENTS roscpp visual_pose_estimation)
#outlet_pose_estimation doesn't have access to the visual_pose_estimation header files

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(outlet_detection src/liboutlet/affine_transform.cpp src/liboutlet/constellation.cpp src/liboutlet/features.cpp src/liboutlet/geometric_hash.cpp src/liboutlet/generalized_hough.cpp src/liboutlet/gh_outlets.cpp src/liboutlet/learning.cpp src/liboutlet/one_way_descriptor_base.cpp src/liboutlet/one_way_descriptor.cpp src/liboutlet/one_way_outlets.cpp src/liboutlet/outlet_detector.cpp src/liboutlet/outlet_model.cpp src/liboutlet/outlet_tuple.cpp src/liboutlet/pca_features.cpp src/liboutlet/planar.cpp)

add_definitions(-D_OUTLET_HR -D_TUNING -D_USE_OUTLET_TUPLE -D_GHT -D_KDTREE -D_GHASH)
find_package(OpenCV REQUIRED)
target_link_libraries(outlet_detection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(outlet_detection visual_pose_estimation_gencpp ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} src/detector.cpp src/estimator.cpp)
target_link_libraries(${PROJECT_NAME} outlet_detection ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} visual_pose_estimation_gencpp)

catkin_package(
    DEPENDS roscpp opencv2
    CATKIN_DEPENDS visual_pose_estimation# TODO
    INCLUDE_DIRS include #TODO include
    LIBRARIES outlet_detection ${PROJECT_NAME}# TODO
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} outlet_detection 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

