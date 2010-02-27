#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from actionlib_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from math import pi



#server actionlib.simple_action_server.SimpleActionServer

def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(5.0)

  # get wall norm
  rospy.loginfo("Get wall norm...")
  wall_norm_goal.look_point.header.frame_id = 'base_link'
  wall_norm_goal.look_point.point.x = -0.14
  wall_norm_goal.look_point.point.y = -0.82
  wall_norm_goal.look_point.point.z = 0.5
  while wall_norm_client.send_goal_and_wait(wall_norm_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Wall norm detection failed, trying again')
    rospy.sleep(2.0)

  # move to joint space position
  rospy.loginfo("Move in joint space...")
  joint_space_goal.trajectory.header.stamp = rospy.Time.now()
  joint_space_goal.trajectory.joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
  joint_space_goal.trajectory.points = [JointTrajectoryPoint([-1.0677069146030997, 0.71083342988028253, -2.1979834079675578, -1.7286367016281583, -4.6274214438187489, -0.48602332590819297, -3.2226035743801464], [], [], rospy.Duration(5.0)),
                                        JointTrajectoryPoint([-1.8962756484688157, 0.70431962080123955, -2.8628145180971059, -0.67036280141457039, -3.7639958239946822, -0.127640351047833, -2.9658574368724415], [], [], rospy.Duration(10.0))]
  if  joint_space_client.send_goal_and_wait(joint_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move in joint space failed')
    server.set_aborted()
    return
  rospy.sleep(2.0)

  # call vision outlet detection
  rospy.loginfo("Detecting outlet...")
  detect_outlet_goal.camera_name = "/forearm_camera_r"
  pose_base_outlet = PoseStampedMath()
  pose_base_outlet.fromEuler(-0.14, -0.82, 0.29, 0, 0, -pi/2)
  detect_outlet_goal.prior = pose_base_outlet.msg
  detect_outlet_goal.prior.header.stamp = rospy.Time.now()
  detect_outlet_goal.prior.header.frame_id = "base_link"
  detect_outlet_goal.wall_normal = wall_norm_client.get_result().wall_norm
  if detect_outlet_client.send_goal_and_wait(detect_outlet_goal, rospy.Duration(30.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Vision outlet detection failed')
    server.set_aborted()
    return

  # return result
  result = DetectOutletResult()
  result.outlet_pose = detect_outlet_client.get_result().outlet_pose
  server.set_succeeded(result)
  rospy.loginfo("Action server goal finished")  


if __name__ == '__main__':
  #Initialize the node
  name = 'detect_outlet'
  rospy.init_node(name)

  # create action clients we use
  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  joint_space_goal = JointTrajectoryGoal()

  wall_norm_client = actionlib.SimpleActionClient('detect_wall_norm', DetectWallNormAction)
  wall_norm_client.wait_for_server()
  wall_norm_goal = DetectWallNormGoal()

  detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
  detect_outlet_client.wait_for_server()
  detect_outlet_goal = VisionOutletDetectionGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, DetectOutletAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
