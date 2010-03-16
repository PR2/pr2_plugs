#!/usr/bin/env python

PKG = 'pr2_plugs_actions'
import roslib; roslib.load_manifest(PKG)
import rospy
import os
import rosparam

if __name__ == '__main__':
  rospy.init_node('load_plug_description', anonymous=True)

  import os

  plug_desc_path = None
  if 'ROBOT_NAME' not in os.environ:
    rospy.logwarn('ROBOT_NAME environment variable not set.  Using generic plug description configuration')
  else:
    filename = os.environ['ROBOT_NAME'] + '_plug_description.yaml'
    plug_desc_path = roslib.packages.find_resource(PKG, filename)
    if not plug_desc_path:
      rospy.logwarn('Could not find robot specific plug description configuration file "%s" in %s' % (filename, PKG))

  if not plug_desc_path:
    plug_desc_path = roslib.packages.find_resource(PKG, 'pr2_plug_description.yaml')
    if not plug_desc_path:
      rospy.logfatal('Could not find plug description configuration file in %s' % PKG)
      sys.exit(1)

  plug_desc_path = plug_desc_path[0]
  rospy.loginfo('Loading plug description from %s' % plug_desc_path)

  try:
    for params, ns in rosparam.load_file(plug_desc_path):
      rosparam.upload_params(ns, params)
  except Exception, ex:
    rospy.logerr('Error loading parameters: ' + str(ex))
    sys.exit(1)

  rospy.spin()
