#!/bin/env python
import roslib; 
from geometry_msgs.msg import *
import rospy
import tf2_ros
import time
from math import sqrt
from baris_utils import *

class TransformListener:
  def __init__(self, useTf = False, withRobot=True, transform_root = 'base_link'):
    self.online = withRobot
    if self.online:
      if not useTf:     # Subscribe to pre-computed transforms
        self.objPoseSub = rospy.Subscriber("/baris/objectTransform", Transform, self.cb)
        self.tr_obj = None
        self.use_tf = False
      else:             # Compute transform using tf2 lookup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.use_tf = useTf
        self.tr_obj = Transform()
    else:               # Robot not online
      self.use_tf = False

    self.object_root = transform_root     # Reference frame for transform
    self.tr = Transform()                 # Required transform either from subscriber or tf2 lookup


  # Set the transform as that provided by subscribed topic
  def cb(self, inTransform):
    self.tr = inTransform

  # Outputs True if the received transform is empty
  def isTrAllZero(self):
    tmp = self.tr.translation.x + self.tr.translation.y + self.tr.translation.z + \
          self.tr.rotation.x + self.tr.rotation.y + self.tr.rotation.z + self.tr.rotation.w
    return tmp == 0

  # Manually fill up transform message
  def fillTransform(self, trans_tuple, rot_tuple):
    self.tr.translation.x = trans_tuple[0]
    self.tr.translation.y = trans_tuple[1]
    self.tr.translation.z = trans_tuple[2]
    
    self.tr.rotation.x = rot_tuple[0]
    self.tr.rotation.y = rot_tuple[1]
    self.tr.rotation.z = rot_tuple[2]
    self.tr.rotation.w = rot_tuple[3]

  # Return the transfrom either from subscribed topic or tf2 lookup
  def getTransform(self):
    if self.use_tf:
      self.updateTransform()
    return self.tr

  # Returns object transform only if object trasform looked up using tf2
  def getObjectTransform(self):
    if not self.use_tf:
      return None
    self.updateTransform()
    return self.tr_obj

  # Avergae out the returned transform
  def getAverageObjectTransform(self, window_size = 10, rate = 10):

    if not self.use_tf:
      return None

    rrate = rospy.Rate(rate)
    self.updateTransform()
    average = rosTransformCopy(self.tr_obj)
    for i in range(0, window_size):
      rrate.sleep()
      self.updateTransform()
      average = rosTransformAdd(average, self.tr_obj, False)#transformAddition(average, self.tr_obj)
    
    average = rosTransformScalarMult(average, 1./(window_size+1))
    average.rotation = quatNormalize(average.rotation, True)
    return average
    
  # Look up transform from root link to main_object. WHAT IS MAIN_OBJECT????
  def updateTransform(self):
    if not self.online:
      return True
    if not self.use_tf:
      print 'Not using tf. Initialize this class with use_tf = True in order to use this function'
      return True
    else:
      try:
        trans = self.tfBuffer.lookup_transform(self.object_root, 'main_object', rospy.Time(0))
        self.tr_obj = trans.transform 
        trans = self.tfBuffer.lookup_transform('kinect_ir_optical_frame', 'main_object', rospy.Time(0))
        self.tr = trans.transform
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'Cannot lookupTransform'
        return False
    return True
