from geometry_msgs.msg import *
from math import sqrt
import sys

def listNorm(inList):
  norm = 0
  for i in range(0,len(inList)):
    norm += inList[i]*inList[i]
  return sqrt(norm)
     
def quatNormalize(inQ, isInStructure = False, isOutStructure = True):
  if isInStructure:
    qOut = [inQ.x,inQ.y,inQ.z,inQ.w]
  else:
    qOut = inQ

  norm = listNorm(qOut)

  if norm == 0:
    raise Exception('Quaternion norm must be non-zero')

  for i in range(0,4):
    qOut[i] /= norm

  if isOutStructure:
    ret = Quaternion()
    ret.x = qOut[0]
    ret.y = qOut[1]
    ret.z = qOut[2]
    ret.w = qOut[3]
    return ret
  else:
    return qOut

def quatFromAngleAxis(inAA, isInStructure = False, isOutStructure = True):
  if isInStructure:
    qOut = [inAA.x,inAA.y,inAA.z,0];
  else:
    qOut = inAA
    qOut.append(0)

  norm = listNorm(inAA);

  if norm > 1:
    raise Exception('Norm of the angle axis cannot be larger than 1') 

  qOut[3] = sqrt(1-norm*norm) #0.47#inTransform.rotation.w

  if isOutStructure:
    ret = Quaternion()
    ret.x = qOut[0]
    ret.y = qOut[1]
    ret.z = qOut[2]
    ret.w = qOut[3]
    return ret
  else:
    return qOut


def transform2pose(inTransform):
  outPose = Pose()

  outPose.position.x = inTransform.translation.x #+ 0.1
  outPose.position.y = inTransform.translation.y #- 0.02
  outPose.position.z = inTransform.translation.z

  outPose.orientation = quatNormalize(inTransform.rotation, True)

  return outPose


def straightLinePoses(startPose, endPose, numSteps = None):
  if numSteps is None:
    numSteps = 3;
  poses = []

  diffPose = rosPoseScalarMult(rosPoseDiff(endPose, startPose, False), 1./numSteps)

  for i in range(0, numSteps+1):
    pose = rosPoseAdd(startPose, rosPoseScalarMult(diffPose,i ))
    poses.append(pose)

  return poses

def rosVec3Add(inVec3a, inVec3b):
  outVec = Vector3()
  outVec.x = inVec3a.x + inVec3b.x
  outVec.y = inVec3a.y + inVec3b.y
  outVec.z = inVec3a.z + inVec3b.z
  return outVec

def rosVec3Diff(inVec3a, inVec3b):
  outVec = Vector3()
  outVec.x = inVec3a.x - inVec3b.x
  outVec.y = inVec3a.y - inVec3b.y
  outVec.z = inVec3a.z - inVec3b.z
  return outVec

#algebraic add!
def rosQuatAdd(inQuat1, inQuat2, shouldNormalize = True):
  outQuat = Quaternion()
  outQuat.x = inQuat1.x + inQuat2.x
  outQuat.y = inQuat1.y + inQuat2.y
  outQuat.z = inQuat1.z + inQuat2.z
  outQuat.w = inQuat1.w + inQuat2.w
  if shouldNormalize:
    outQuat = quatNormalize(outQuat, True)
  return outQuat

#algebraic diff!
def rosQuatDiff(inQuat1, inQuat2, shouldNormalize = True):
  outQuat = Quaternion()
  outQuat.x = inQuat1.x - inQuat2.x
  outQuat.y = inQuat1.y - inQuat2.y
  outQuat.z = inQuat1.z - inQuat2.z
  outQuat.w = inQuat1.w - inQuat2.w
  if shouldNormalize:
    outQuat = quatNormalize(outQuat, True)
  return outQuat

def rosPoseAdd(pose1, pose2, shouldNormalizeQuats = True):
  pose = Pose()
  pose.position = rosVec3Add(pose1.position, pose2.position)
  pose.orientation = rosQuatAdd(pose1.orientation, pose2.orientation, shouldNormalizeQuats)
  return pose

def rosPoseDiff(pose1, pose2, shouldNormalizeQuats = True):
  pose = Pose()
  pose.position = rosVec3Diff(pose1.position, pose2.position)
  pose.orientation = rosQuatDiff(pose1.orientation, pose2.orientation, shouldNormalizeQuats)
  return pose

def rosTransformAdd(tr1, tr2, shouldNormalizeQuats = True):
  tr = Transform()
  tr.translation = rosVec3Add(tr1.translation, tr2.translation)
  tr.rotation = rosQuatAdd(tr1.rotation, tr2.rotation, shouldNormalizeQuats)
  return tr

def rosTransformDiff(tr1, tr2, shouldNormalizeQuats = True):
  tr = Transform()
  tr.translation = rosVec3Diff(tr1.translation, tr2.translation)
  tr.rotation = rosQuatDiff(tr1.rotation, tr2.rotation, shouldNormalizeQuats)
  return tr

def rosVec3ScalarMult(inVec3, inScalar):
  outVec = Vector3()
  outVec.x = inVec3.x*inScalar
  outVec.y = inVec3.y*inScalar
  outVec.z = inVec3.z*inScalar
  return outVec

#not that by itself the below does not make sense
def rosQuatScalarMult(inQuat, inScalar):
  outQuat = Quaternion()
  outQuat.x = inQuat.x*inScalar
  outQuat.y = inQuat.y*inScalar
  outQuat.z = inQuat.z*inScalar
  outQuat.w = inQuat.w*inScalar
  return outQuat

def rosPoseScalarMult(inPose, scalar):
  pose = Pose()

  pose.position = rosVec3ScalarMult(inPose.position, scalar)
  pose.orientation = rosQuatScalarMult(inPose.orientation, scalar)

  return pose

def rosTransformScalarMult(inTransfrom, scalar):
  tr = Transform()

  tr.translation = rosVec3ScalarMult(inTransfrom.translation, scalar)
  tr.rotation = rosQuatScalarMult(inTransfrom.rotation, scalar)

  return tr

def rosPoseCopy(inPose):
  outPose = Pose()
  outPose.position = rosVector3Copy(inPose.position)
  outPose.orientation = rosQuatCopy(inPose.orientation)
  return outPose

def rosTransformCopy(inTr):
  outTr = Transform()
  outTr.translation = rosVector3Copy(inTr.translation)
  outTr.rotation = rosQuatCopy(inTr.rotation)
  return outTr

def rosVector3Copy(inVec3):
  outVec3 = Vector3()
  outVec3.x = inVec3.x
  outVec3.y = inVec3.y
  outVec3.z = inVec3.z
  return outVec3

def rosQuatCopy(inQuat):
  outQuat = Quaternion()
  outQuat.x = inQuat.x
  outQuat.y = inQuat.y
  outQuat.z = inQuat.z
  outQuat.w = inQuat.w  
  return outQuat

if __name__ == '__main__':
  targetPose = Pose()
  targetPose.position.x =  1.2119346268
  targetPose.position.y =  0.075
  targetPose.position.z =  1.03819858085

  targetPose.orientation = quatFromAngleAxis([-0.5094,0.5094,0.5094])

  print targetPose
  
  curPose = rosPoseCopy(targetPose)
  targetPose.position.z = max(0.91, targetPose.position.z-0.12)
  poses = straightLinePoses(targetPose, curPose)

  curPose = rosPoseCopy(targetPose)
  targetPose.position.z += 0.20
