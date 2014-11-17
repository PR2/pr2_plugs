
try:
    import roslib; roslib.load_manifest('pr2_plugs_actions')
except:
    pass

import rospy 
import tf
try:
    import tf2 # groovy
except:
    import tf2_py as tf2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
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
            TFUtil.listener = tf2_ros.buffer_client.BufferClient('tf2_buffer_server_plugs')
            TFUtil.listener.wait_for_server()
            TFUtil.broadcaster = tf.TransformBroadcaster()
            TFUtil.initialized = True

            TFUtil.broadcast_lock = threading.Lock()
        
    @staticmethod
    def wait_and_transform(frame_id, pose):
        try:
            result = TFUtil.listener.transform(pose, frame_id, rospy.Duration(5.0))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.TimeoutException, tf2.ExtrapolationException, tf2.TransformException) as e:
            rospy.logerr("tf2_ros BufferClient threw an exception: %s, trying again"%str(e))
            result = TFUtil.listener.transform(pose, frame_id, rospy.Duration(5.0))
        return result

    @staticmethod
    def wait_and_lookup(parent_frame_id, child_frame_id, time=None):
        if time == None:
            time = rospy.Time.now()

        ps = PoseStamped()
        ps.header.stamp = time
        ps.header.frame_id = child_frame_id
        ps.pose.position.x = 0.0
        ps.pose.position.y = 0.0
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        try:
            result = TFUtil.listener.transform(ps, parent_frame_id, rospy.Duration(2.0))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.TimeoutException, tf2.ExtrapolationException, tf2.TransformException) as e:
            rospy.logerr("tf2_ros BufferClient threw an exception: %s, trying again"%str(e))
            result = TFUtil.listener.transform(ps, parent_frame_id, rospy.Duration(2.0))
        return result 

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

