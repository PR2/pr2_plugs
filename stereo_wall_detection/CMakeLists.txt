cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_gensrv ()

rosbuild_add_executable (wall_extractor src/wall_extractor.cpp)
rosbuild_add_executable (service_trigger src/service_trigger.cpp)
target_link_libraries (wall_extractor cloud_geometry ANN cloud_kdtree sample_consensus)


rosbuild_add_library (cloud_kdtree
                       src/cloud_kdtree/kdtree_ann.cpp
                      )

rosbuild_add_library (ANN
                       src/ANN/brute.cpp
                       src/ANN/bd_fix_rad_search.cpp
                       src/ANN/bd_pr_search.cpp
                       src/ANN/bd_search.cpp
                       src/ANN/bd_tree.cpp
                       src/ANN/kd_dump.cpp
                       src/ANN/kd_fix_rad_search.cpp
                       src/ANN/kd_pr_search.cpp
                       src/ANN/perf.cpp
                       src/ANN/ANN.cpp
                       src/ANN/kd_search.cpp
                       src/ANN/kd_split.cpp
                       src/ANN/kd_tree.cpp
                       src/ANN/kd_util.cpp
                      )

rosbuild_add_library (cloud_geometry
                       src/cloud_geometry/angles.cpp
                       src/cloud_geometry/areas.cpp
                       src/cloud_geometry/distances.cpp
                       src/cloud_geometry/intersections.cpp
                       src/cloud_geometry/nearest.cpp
                       src/cloud_geometry/point.cpp
                       src/cloud_geometry/projections.cpp
                       src/cloud_geometry/statistics.cpp
                       src/cloud_geometry/transforms.cpp
                      )

rosbuild_add_library (sample_consensus
                       src/sample_consensus/sac.cpp
                       src/sample_consensus/sac_model.cpp
                       src/sample_consensus/msac.cpp
                       src/sample_consensus/sac_model_plane.cpp
                       src/sample_consensus/sac_model_normal_plane.cpp
                      )


target_link_libraries (cloud_kdtree ANN)
target_link_libraries (sample_consensus cloud_geometry)
target_link_libraries (cloud_geometry cloud_kdtree)
