
import roslib; roslib.load_manifest('pr2_plugs_actions')

import rospy 
import tf
from std_srvs.srv import *

import threading

__all__ = ['TFUtil']

class TFUtil():
    initialized = False

    listener = None
    broadcaster = None

    broadcast_lock = None
    broadcast_list = {}
    broadcast_threads = {}
    
    def __init__(self):
        if not TFUtil.initialized:
            TFUtil.listener = tf.TransformListener(True, rospy.Duration(60.0))    
            TFUtil.broadcaster = tf.TransformBroadcaster()
            TFUtil.initialized = True

            TFUtil.broadcast_lock = threading.Lock()
        
    @staticmethod
    def wait_and_transform(frame_id,pose):
        try:
            TFUtil.listener.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
        except rospy.ServiceException, ex:
            rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
            raise ex
        return TFUtil.listener.transformPose(frame_id, pose)

    @staticmethod
    def wait_and_lookup(parent_frame_id, child_frame_id, time=None):
        if time == None:
            time = rospy.Time.now()
        try:
            TFUtil.listener.waitForTransform(parent_frame_id, child_frame_id, time, rospy.Duration(2.0))
        except rospy.ServiceException, ex:
            rospy.logerr('Could not transform between %s and %s' % (parent_frame_id,child_frame_id))
            raise ex
        return TFUtil.listener.lookupTransform(parent_frame_id, child_frame_id, time)


    @staticmethod
    def broadcast_transform(frame_id, pose,time = None, period = rospy.Duration(0.1)):
        if frame_id in TFUtil.broadcast_list:
            with TFUtil.broadcast_lock:
                TFUtil.broadcast_list[frame_id] = (pose, time, period)
        else:
            broadcast_thread = threading.Thread(
                    target=TFUtil.broadcast_loop,
                    name='TFUtil Broadcaster: \'%s\'' % frame_id,
                    args=[frame_id])
            TFUtil.broadcast_list[frame_id] = (pose,time,period)
            TFUtil.broadcast_threads[frame_id] = broadcast_thread
            broadcast_thread.start()

    @staticmethod
    def stop_broadcasting_transform(frame_id):
        if frame_id in TFUtil.broadcast_list:
            with TFUtil.broadcast_lock:
                del TFUtil.broadcast_list[frame_id]

    @staticmethod
    def broadcast_loop(frame_id):
        while not rospy.is_shutdown():
            with TFUtil.broadcast_lock:
                if frame_id not in TFUtil.broadcast_list:
                    break
                pose, time, period = TFUtil.broadcast_list[frame_id]
                if time is None:
                    time = rospy.Time.now()
                TFUtil.broadcaster.sendTransform(
                        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                        [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w],
                        time,
                        frame_id,
                        pose.header.frame_id)
            rospy.sleep(period)

