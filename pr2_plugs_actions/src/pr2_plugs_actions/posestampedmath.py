PKG = 'pr2_plugs_actions'
import roslib
roslib.load_manifest(PKG)

from geometry_msgs.msg import *
from tf import transformations
import tf
import rospy
from math import pi
import numpy

class PoseStampedMath(object):
    
    def __init__(self, msg=None):
        if msg == None:
            msg = PoseStamped()
        self.msg = msg

    def __mul__(self, other):
#        print self.asMatrix()
#        print other.asMatrix()
        m = numpy.dot(self.asMatrix(), other.asMatrix())
        msg = PoseStamped()
        msg.pose.position.x = m[0,3]
        msg.pose.position.y = m[1,3]
        msg.pose.position.z = m[2,3]
        tmp = transformations.quaternion_from_matrix(m)
        msg.pose.orientation.x = tmp[0]
        msg.pose.orientation.y = tmp[1]
        msg.pose.orientation.z = tmp[2]
        msg.pose.orientation.w = tmp[3]
        msg.header.frame_id = self.msg.header.frame_id
        return PoseStampedMath(msg)

    def inverse(self):
        inv = numpy.linalg.inv(self.asMatrix())
        msg = PoseStamped()
        msg.pose.position.x = inv[0,3]
        msg.pose.position.y = inv[1,3]
        msg.pose.position.z = inv[2,3]
        tmp = transformations.quaternion_from_matrix(inv)
        msg.pose.orientation.x = tmp[0]
        msg.pose.orientation.y = tmp[1]
        msg.pose.orientation.z = tmp[2]
        msg.pose.orientation.w = tmp[3]
        return PoseStampedMath(msg)

    def asMatrix(self):
        translation = (self.msg.pose.position.x, self.msg.pose.position.y, self.msg.pose.position.z)
        rotation = (self.msg.pose.orientation.x, self.msg.pose.orientation.y, self.msg.pose.orientation.z, self.msg.pose.orientation.w)
        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))        

    def fromEuler(self, x, y, z, Rx, Ry, Rz):
        tmp = transformations.quaternion_from_euler(Rx, Ry, Rz)
        self.msg.pose.orientation.x = tmp[0]
        self.msg.pose.orientation.y = tmp[1]
        self.msg.pose.orientation.z = tmp[2]
        self.msg.pose.orientation.w = tmp[3]
        self.msg.pose.position.x = x
        self.msg.pose.position.y = y
        self.msg.pose.position.z = z
        return self

    def toEuler(self):
      tmp = transformations.euler_from_quaternion((self.msg.pose.orientation.x, self.msg.pose.orientation.y, self.msg.pose.orientation.z, self.msg.pose.orientation.w))
      return self.msg.pose.position.x, self.msg.pose.position.y, self.msg.pose.position.z, tmp[0], tmp[1], tmp[2]
    
    def fromTf(self, tf):
        position, quaternion = tf
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]
        self.msg.pose.position.x = position[0]
        self.msg.pose.position.y = position[1]
        self.msg.pose.position.z = position[2]
        return self

# create a
a = PoseStampedMath()
a.fromEuler(5,0,0,  0, pi/2, 0)

# create b
b = PoseStampedMath()
transformer = tf.Transformer(True, rospy.Duration(10.0))
m = TransformStamped()
m.header.frame_id = 'wim'
m.child_frame_id = 'james'
m.transform.translation.x = 2.71828183
m.transform.rotation.w = 1.0
transformer.setTransform(m)
b.fromTf(transformer.lookupTransform('wim', 'james', rospy.Time(0)))
a.inverse()
#print a * b
