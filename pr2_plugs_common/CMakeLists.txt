project(pr2_plugs_common)
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs geometry_msgs actionlib pr2_controllers_msgs std_srvs tf2_geometry_msgs)
find_package(orocos_kdl REQUIRED)
find_library(orocos_kdl_LIBRARY NAMES ${orocos_kdl_LIBRARIES} PATHS)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)
include_directories(${BFL_INCLUDE_DIRS})
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS})
link_directories(${BFL_LINK_DIRS})



add_executable(pose_to_tf src/pose_to_tf.cpp)
target_link_libraries(pose_to_tf ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${BFL_LIBRARIES})
add_dependencies(pose_to_tf ${PROJECT_NAME} bfl_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(detector_stub src/detector_stub.cpp)
target_link_libraries(detector_stub ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${BFL_LIBRARIES})
add_dependencies(detector_stub ${PROJECT_NAME} bfl_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(detector_filter src/detector_filter.cpp)
add_dependencies(detector_filter ${PROJECT_NAME})
target_link_libraries(detector_filter ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${BFL_LIBRARIES})

add_executable(covariance_inserter src/covariance_inserter.cpp)
target_link_libraries(covariance_inserter ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${BFL_LIBRARIES})
add_dependencies(covariance_inserter ${PROJECT_NAME} bfl_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(joint_space_move_test EXCLUDE_FROM_ALL test/joint_space_move_test.cpp include/pr2_plugs_common/joint_space_move.h)

catkin_package(
    DEPENDS roscpp tf bfl std_msgs geometry_msgs actionlib pr2_controllers_msgs std_srvs BFL
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include# TODO include
    LIBRARIES # TODO
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
