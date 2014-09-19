cmake_minimum_required(VERSION 2.8.3)
project(pr2_plugs_actions)

find_package(catkin REQUIRED COMPONENTS roscpp_serialization actionlib roscpp rospy  actionlib_msgs geometry_msgs pr2_controllers_msgs pr2_controller_manager sensor_msgs tf tf_conversions tf2_ros tf2_geometry_msgs pr2_msgs std_srvs move_base_msgs dynamic_reconfigure pr2_common_action_msgs pr2_plugs_common pr2_plugs_msgs stereo_wall_detection checkerboard_pose_estimation outlet_pose_estimation image_transport joint_trajectory_action_tools pr2_arm_move_ik costmap_2d base_local_planner pr2_image_snapshot_recorder smach smach_ros cv_bridge rostest)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${pcl_INCLUDE_DIRS})

add_executable(vision_detect_plug src/vision_detect_plug.cpp)
target_link_libraries(vision_detect_plug ${catkin_LIBARIES} ${OpenCV_LIBRARIES} ${roscpp_serialization_LIBRARIES} ${roscpp_LIBRARIES} ${actionlib_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES} ${checkerboard_pose_estimation_LIBRARIES})
add_dependencies(vision_detect_plug roscpp_serialization checkerboard_pose_estimation)


add_executable(vision_detect_outlet src/vision_detect_outlet.cpp)
target_link_libraries(vision_detect_outlet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${outlet_pose_estimation_LIBRARIES} ${visual_pose_estimation})
add_dependencies(vision_detect_outlet outlet_pose_estimation visual_pose_estimation)


add_executable(align_base src/align_base.cpp)
target_link_libraries(align_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_base_omnidirectional src/move_base_omnidirectional.cpp)
target_link_libraries(move_base_omnidirectional ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#Tests
add_rostest_gtest(fetch_and_stow_plug_test test/action_server_tests/fetch_and_stow_plug_test.launch test/action_server_tests/fetch_and_stow_plug_test.cpp)
target_link_libraries(fetch_and_stow_plug_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

#Tests
add_rostest_gtest(detect_plug_test test/action_server_tests/detect_plug_test.launch test/action_server_tests/detect_plug_test.cpp)
add_executable(detect_plug_test EXCLUDE_FROM_ALL test/action_server_tests/detect_plug_test.cpp)
target_link_libraries(detect_plug_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_rostest_gtest(detect_wall_norm_test test/action_server_tests/detect_wall_norm_test.launch test/action_server_tests/detect_wall_norm_test.cpp)
target_link_libraries(detect_wall_norm_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_rostest_gtest(wiggle_plug_test test/action_server_tests/wiggle_plug_test.launch test/action_server_tests/wiggle_plug_test.cpp)
target_link_libraries(wiggle_plug_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_package(
    DEPENDS actionlib roscpp rospy actionlib_msgs geometry_msgs pr2_controllers_msgs pr2_controller_manager sensor_msgs tf kdl tf_conversions tf2_ros tf2_geometry_msgs pr2_msgs std_srvs move_base_msgs dynamic_reconfigure pr2_common_action_msgs pr2_plugs_common pr2_plugs_msgs stereo_wall_detection checkerboard_pose_estimation outlet_pose_estimation image_transport joint_trajectory_action_tools pr2_arm_move_ik costmap_2d base_local_planner pr2_image_snapshot_recorder smach smach_ros cv_bridge opencv2
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES # TODO
)

install(TARGETS fetch_and_stow_plug_test detect_plug_test detect_wall_norm_test vision_detect_plug vision_detect_outlet align_base move_base_omnidirectional 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/battery.py scripts/detect_plug.py scripts/outlet_server.py scripts/recharge_toplevel.py scripts/tf_publish.py scripts/wiggle_plug.py scripts/detect_outlet.py scripts/detect_wall_norm.py scripts/plug_in.py scripts/recharge_web_adaptor.py scripts/vision_detect_outlet.py scripts/detect_plug_on_base.py scripts/fetch_plug.py scripts/recharge_application.py scripts/stow_plug.py scripts/vision_detect_plug.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config/ launch/ test/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
