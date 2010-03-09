#!/usr/bin/env python
#Wiggle the plug - designed for the last few CM in or out

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
import tf
import geometry_msgs.msg
import visualization_msgs.msg
import copy
import math
from sensor_msgs.msg import JointState

class PlugWiggleServer:
  def __init__(self, name):
    self.tf = tf.TransformListener()
    self.name = name

    #TODO Create action client to IK action

    self.marker_pub = rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker)

    self.ik_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
    self.ik_client.wait_for_server()

    self.ik_seed = None
    self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

    self.server = actionlib.simple_action_server.SimpleActionServer(self.name, WigglePlugAction)
    self.server.register_goal_callback(self.goalCB)


  def wait_and_transform(self,frame_id,pose):
    try:
      self.tf.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return self.tf.transformPose(frame_id, pose)

  def joint_state_cb(self, msg):
    if self.ik_seed:
      return
    print "Got joint_state"
    ik_seed = sensor_msgs.msg.JointState() 
    for j, p in zip(msg.name, msg.position):
      if j[0] == 'r' and j[2] != 'g': #Terrible hack to get arm joints     
        ik_seed.name.append(j)
        ik_seed.position.append(p)
    self.ik_seed = ik_seed
    #self.joint_state_sub.unregister()

  def goalCB(self):
    #Accept goal
    goal = self.server.accept_new_goal()
    #TODO Get the current cartesian and joint pose
    #Find initial pose
    initial_pose = geometry_msgs.msg.PoseStamped()
    initial_pose.header.frame_id = 'r_wrist_roll_link'
    initial_pose = self.tf.transformPose('base_link', initial_pose)

    #Find initial pose of plug
    initial_plug_pose = geometry_msgs.msg.PoseStamped()
    initial_plug_pose = self.wait_and_transform('base_link', goal.initial_plug_pose)

    #Transform desired velocity
    travel_velocity = geometry_msgs.msg.Vector3Stamped()
    travel_velocity.header.frame_id = 'r_wrist_roll_link'
    travel_velocity.vector = goal.travel
    travel_velocity = self.tf.transformVector3('base_link', travel_velocity)
    travel_velocity.vector.x /= goal.move_duration.to_seconds()
    travel_velocity.vector.y /= goal.move_duration.to_seconds()
    travel_velocity.vector.z /= goal.move_duration.to_seconds()
    #Transform desired displacement
    offset_vector = geometry_msgs.msg.Vector3Stamped()
    offset_vector.header.frame_id = 'r_wrist_roll_link'
    offset_vector.vector = goal.offset
    offset_vector = self.tf.transformVector3('base_link', offset_vector)

    #Publish initial position and desired push as an arrow
    self.publish_marker(initial_pose)
 
    start = rospy.Time.now()
    rate = rospy.Rate(10.0);
    
    self.ik_seed = None
    while(self.ik_seed == None):
      print "Waiting for ik_seed"
      rate.sleep()

    t = 0
    while(rospy.Time.now() < start + goal.move_duration):
      #Compute original pose + travel * distance
      goal_pose = copy.deepcopy(initial_pose)
      #t = (rospy.Time.now() - start).to_seconds() # time from start
      c = math.sin(t * 2 * math.pi / goal.period.to_seconds()) # scale of offset
      v = travel_velocity.vector
      o = offset_vector.vector
      goal_pose.pose.position.x += v.x * t + o.x * c;
      goal_pose.pose.position.y += v.y * t + o.y * c;
      goal_pose.pose.position.z += v.z * t + o.z * c;
      self.publish_marker(goal_pose)
      self.command_arm_ik(goal_pose, rospy.Duration(0.5))
      t = t + 0.5;
      #rate.sleep()
      #Find initial pose of plug
      current_plug_pose = geometry_msgs.msg.PoseStamped()
      initial_plug_pose.header.stamp = rospy.Time.now()
      current_plug_pose = self.wait_and_transform('base_link', goal.initial_plug_pose)
      #If we've wiggled more than abort_threshold out of line, assume we were not in the socket and abort.
      x_error = current_plug_pose.pose.position.x - initial_plug_pose.pose.position.x
      y_error = current_plug_pose.pose.position.y - initial_plug_pose.pose.position.y
      z_error = current_plug_pose.pose.position.z - initial_plug_pose.pose.position.z
      rospy.loginfo("Wiggle errors: %f, %f, %f", x_error, y_error, z_error)
      #If abort_threshold isn't set, always succeed.
      if(goal.abort_threshold > 0):
        if(math.fabs(y_error) + math.fabs(z_error) > goal.abort_threshold):
          rospy.logerr("During wiggle the plug moved more than threshold of %f.  This probably means we weren't in the socket."%goal.abort_threshold)
          self.server.set_aborted(WigglePlugResult())
          return
          
    self.server.set_succeeded(WigglePlugResult())

      

  def publish_marker(self, pose):
    marker = visualization_msgs.msg.Marker()
    marker.header = copy.deepcopy(pose.header)
    marker.pose = copy.deepcopy(pose.pose)
    marker.ns = "plugs"
    marker.id = 0
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color.a = 0.5
    marker.color.g = 1.0
    marker.lifetime = rospy.Duration(2.0)
    self.marker_pub.publish(marker)

  def command_arm_ik(self, pose, dt):
    goal = PR2ArmIKGoal()
    goal.pose = pose
    goal.ik_seed = self.ik_seed
    goal.ik_timeout = rospy.Duration(0.1)
    goal.move_duration = dt
    print "Not actually sending IK goal"
    print goal
    self.ik_client.send_goal(goal)
    self.ik_client.wait_for_result()
  

if __name__ == '__main__':
  #Initialize the node
  name = 'wiggle_plug'
  rospy.init_node(name)
  wiggle_server = PlugWiggleServer(name)
  rospy.loginfo('%s: Action server running', name)
  rospy.spin()
  print "Spin returned"
