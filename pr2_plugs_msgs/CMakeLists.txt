cmake_minimum_required(VERSION 2.8.3)
project(pr2_plugs_msgs)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs geometry_msgs sensor_msgs std_msgs)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  # TODO: List your msg files here
  OutletPoseArray.msg  
  OutletPose.msg  
  RechargeCommand.msg  
  RechargeState.msg
)
add_service_files(
  FILES
  # TODO: List your msg files here
  GetOutlets.srv  
  RechargeRequest.srv
)

add_action_files(
  FILES
  AlignBase.action   
  Empty.action    
  VisionOutletDetection.action
  DetectOutlet.action         
  FetchPlug.action  
  VisionPlugDetection.action
  DetectPlugInGripper.action  
  PlugIn.action    
  WigglePlug.action
  DetectPlugOnBase.action     
  Recharge.action
  DetectWallNorm.action      
  StowPlug.action
)

generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
    DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS actionlib_msgs geometry_msgs sensor_msgs std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


install(FILES rosdoc.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

