#!/bin/env python
import roslib; 
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import rospy
from math import *

#returns the plausible root of sin(x)*c1+cos(x)*c2 = c3
def trigSolution(params):
  y = atan(abs(params[1]/params[0]))
  h = params[2]/sqrt(params[0]*params[0]+params[1]*params[1])
  if(params[0] > 0 and params[1] < 0):
    return asin(h) + y
  if(params[0] < 0 and params[1] < 0):
    return asin(-h) - y
  if(params[0] < 0 and params[1] > 0):
    return asin(-h) + y
  else:
    return asin(h) - y

#TODO: decouple IK functionality from ROS stuff

class PanTilt:
  def __init__(self, withRobot = True):

    self.online = withRobot

    # Subscribe and Publish to Pan-Tilt Topics
    if self.online:
      self.pubTilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
      self.pubPan  = rospy.Publisher('/pan_controller/command', Float64, queue_size=10)

      self.subTilt = rospy.Subscriber("/tilt_controller/state", JointState, self.cb_tilt)
      self.subPan = rospy.Subscriber("/pan_controller/state", JointState, self.cb_pan)

    # Initialize parameters
    self.tilt_pos = 0.0
    self.pan_pos = 0.0
    self.pt_params = [-0.0985, (0.03325+0.06175)/2, -0.04201 ,0.0245]

    # whether to calculate the joint position for the imaginary link at the end for ik
    self.calc_ik_im_trans = False


  # Sweep the Pan and Tilt joints as per specified rate and repetitions
  def adjust(self, pos, repetitions = 10, rate = 10):
    if self.online:
      ros_rate = rospy.Rate(rate)
      for i in range(0,repetitions):
        self.pubPan.publish(pos[0])
        self.pubTilt.publish(pos[1])
        ros_rate.sleep()


  # Get current tilt
  def cb_tilt(self, js):
    self.tilt_pos = js.current_pos


  # Get current pan
  def cb_pan(self, js):
    self.pan_pos = js.current_pos


  # Get transform from root to object (NOTE: only translational part of the transform considered)
  def baseToObject(self, pos, theta = None):
    if theta is None:
      theta = [self.pan_pos, self.tilt_pos]

    X = (self.pt_params[1] + pos[0])*cos(theta[0]) + (self.pt_params[2] + pos[1])*sin(theta[0])*sin(theta[1]) - (self.pt_params[3] + pos[2])*sin(theta[0])*cos(theta[1])
    Y =  self.pt_params[0]                         + (self.pt_params[2] + pos[1])              *cos(theta[1]) + (self.pt_params[3] + pos[2])              *sin(theta[1])
    Z = (self.pt_params[1] + pos[0])*sin(theta[0]) - (self.pt_params[2] + pos[1])*cos(theta[0])*sin(theta[1]) + (self.pt_params[3] + pos[2])*cos(theta[0])*cos(theta[1])
    return [X,Y,Z]

  #this does not work for negative z!
  def headIK(self, pos):
    t1 = trigSolution([pos[2], pos[0], self.pt_params[1]])
    t2 = trigSolution([sin(t1)*self.pt_params[1]-pos[2], cos(t1)*(pos[1]-self.pt_params[0]), cos(t1)*self.pt_params[2]])

    if self.calc_ik_im_trans:
      Dx = -pos[0] + (self.pt_params[1]*cos(t1) + self.pt_params[2]*sin(t1)*sin(t2))
      Dy =  pos[1] - (self.pt_params[0]         + self.pt_params[2]        *cos(t2))
      Dz =  pos[2] - (self.pt_params[1]*sin(t1) - self.pt_params[2]*cos(t1)*sin(t2))
      #This is the most computationally expensive way to calculate z3 but it avoids checking for div by 0
      #Also you do not even need to calculate this :)
      z3 = sqrt(Dx*Dx+Dy*Dy+Dz*Dz)-self.pt_params[3]
      return [t1,t2,z3]
    else:
      return [t1,t2]
