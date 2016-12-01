import roslib; 
from std_msgs.msg import *
from geometry_msgs.msg import *
import rospy
import time

from utilities.kinect_pantilt import PanTilt
from utilities.transform_listener import TransformListener

from math import sqrt


#  Define ObjectSearch class to find specified object on table
class ObjectSearch:
  def __init__(self, withRobot=True, tr_root = 'base_link'):
    
    self.online = withRobot

    # Initialize Pan-Tilt Controller to search the object
    self.pantilt = PanTilt(self.online)

    # Initialize transform listener from utilities
    self.tf = TransformListener(useTf=True,  withRobot=self.online, transform_root = tr_root)

    # Initialize other search parameters for Pan-Tilt
    self.searchPattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]
    self.pt_rate   = 20
    self.pt_repeat = 20


  # Define objectSearch method
  def objectSearch(self):
    if not self.online:  # Do not search if the withRobot flag is False
      return True

    self.tf.updateTransform()  # Get update transform from the transform listener
    objectFound = False
    for i in range(0, len(self.searchPattern)):
      self.pantilt.adjust(self.searchPattern[i], self.pt_rate, self.pt_repeat)
      time.sleep(0.5)
      if(self.tf.updateTransform()):
        print self.tf.tr
        if not(self.tf.isTrAllZero()):   # object found if object transform is non-empty
          objectFound = True
          break
    return objectFound


  # Look at object (NOTE: only translational part of the transform considered)
  def directLookAtObject(self):
    if not self.online:
      return 
    self.tf.updateTransform()
    pos = [self.tf.tr.translation.x, self.tf.tr.translation.y, self.tf.tr.translation.z]
    posInBase = self.pantilt.baseToObject(pos)
    h_ik = self.pantilt.headIK(posInBase)
    self.pantilt.adjust([h_ik[0], h_ik[1]],3,3)
    print h_ik


if __name__ == '__main__':
  rospy.init_node('object_search')
  os = ObjectSearch()

  if(os.objectSearch()):
    os.directLookAtObject()
    os.tf.updateTransform()
  else:
    print 'no obj found'
    os.pantilt.adjust([0,0],10,10)

