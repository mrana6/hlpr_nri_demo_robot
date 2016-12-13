#!/usr/bin/env python

# Importing ROS modules
import roslib
import rospy
import actionlib
import time
import geometry_msgs.msg
import std_srvs.srv

# Importing RAIL modules
import rail_manipulation_msgs.srv
import rail_manipulation_msgs.msg
from utilities.kinect_pantilt import PanTilt

# Importing HLPR modules


class ObjectSearcher:
    def __init__(self, requestedObjectNameIn='BANANA_ARTIFICIAL'):
        rospy.wait_for_service('/rail_segmentation/segment')
        self.segmentationService = rospy.ServiceProxy('/rail_segmentation/segment', std_srvs.srv.Empty)
        self.recognitionSub = rospy.Subscriber("/object_recognition_listener/recognized_objects",
                                               rail_manipulation_msgs.msg.SegmentedObjectList,
                                               self.recognitionCallback)

        self.requestedObjectName = requestedObjectNameIn
        self.recognizedObject = None
        self.panTilt = PanTilt(True)
        self.searchPattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]
        self.pt_rate = 20
        self.pt_repeat = 20

    def findObject(self):
        for i in range(0, len(self.searchPattern)):
            self.panTilt.adjust(self.searchPattern[i], self.pt_rate, self.pt_repeat)
            time.sleep(5.0)
            self.getRecognizedObject()
            time.sleep(1.0)
            if self.recognizedObject is not None:
                print "Got it!"
                break

    def callSegmentationService(self):
        try:
            self.segmentationService()
            rospy.loginfo('Segmentation Service Call Succeeded')
        except:
            rospy.loginfo('Segmentation Service Call Failed')

    def getRecognizedObject(self):
        try:
            self.callSegmentationService()
            rospy.wait_for_message('/object_recognition_listener/recognized_objects',
                                   rail_manipulation_msgs.msg.SegmentedObjectList, timeout=2)
        except:
            rospy.loginfo('Time Out - Did not receive recognized object list')

    def recognitionCallback(self, receivedObjectList):
        if receivedObjectList is None:
            rospy.loginfo('Received Empty List')
        else:
            rospy.loginfo('Requested Object Name: %s' % self.requestedObjectName)
            for receivedObject in receivedObjectList.objects:
                rospy.loginfo('Received Object Name: %s' % receivedObject.name)
                if receivedObject.name == self.requestedObjectName and receivedObject.recognized:
                    self.recognizedObject = receivedObject


class ObjectPicker:
    def __init__(self, graspsIn=None):
        self.receivedGrasps = graspsIn
        self.motionPlanningSuccess = False
        self.pickUpSuccess = False

    def executePickUp(self):
        if self.receivedGrasps is not None:
            rospy.loginfo('Number of Grasps Received: %d' % len(self.receivedGrasps))
            rospy.loginfo('Choosing the first Grasp')
            pickupClient = actionlib.SimpleActionClient('hlpr_manipulation/common_actions/pickup',
                                                         rail_manipulation_msgs.msg.PickupAction)
            pickupClient.wait_for_server()
            pickupGoal = rail_manipulation_msgs.msg.PickupGoal(pose=self.receivedGrasps[0].grasp_pose,
                                                                lift=True,
                                                                verify=False,
                                                                attachObject=True)
            pickupClient.send_goal(pickupGoal)
            pickupClient.wait_for_result(rospy.Duration.from_sec(30.0))
            pickupResult = pickupClient.get_result()
            if pickupResult is not None:
                if not pickupResult.executionSuccess:
                    rospy.loginfo('Motion Planning Failed')
                else:
                    rospy.loginfo('Motion Planning Succeeded')
                    self.motionPlanningSuccess = True
                    pickupClient.wait_for_result(rospy.Duration.from_sec(20.0))
                    pickupResult = pickupClient.get_result()
                    if not pickupResult.success:
                        rospy.loginfo('Pick Up Failed')
                    else:
                        rospy.loginfo('Pick Up Succeeded')
                        self.pickUpSuccess = True
            else:
                rospy.loginfo('Timed Out - No pickup result received')


class ArmMover:
    def __init__(self, armActionIn = 'Tuck'):
        self.jointPlanningSuccess = False
        self.armAction = armActionIn
        self.armClient = actionlib.SimpleActionClient('hlpr_moveit_wrapper/move_to_joint_pose',
                                                      rail_manipulation_msgs.msg.MoveToJointPoseAction)
        self.armClient.wait_for_server()
        self.tuckJointPose = [-1.49870508871867, 1.4097266130121957, 0.4402384379484289,
                              -2.4559040403532384, -3.0710284720826175, 0.7225042853038645] #[-1.9, 1.4, 0.53, -2.5, -3.0, 0.72]
        self.handOffJointPose = [-1.60, 2.60, 1.20, -3.14, 1.50, 1.20]
        self.readyJointPose = [-1.5708, 1.5708, 1.5708, 0.5, 0.5, 1.0]
        #self.readyJointPose = [-1.70, 1.8, 1.3, 0.2, 0.9, 1.3]
        self.retractJointPose = [-1.70, 2.00, 1.00, -2.30, 2.00, 0.90]
        self.upperTuckPose = [-1.4735025300588136, 1.0, 0.8,
                              -2.084300123965141, 1.4451027133522634, 1.3150955778920286]

        self.motionDict = {'Tuck':self.tuckJointPose, 'HandOff':self.handOffJointPose,
                           'Ready':self.readyJointPose, 'Retract':self.readyJointPose,
                           'upperTuck':self.upperTuckPose}


    def executeArmMotion(self):
        armGoal = rail_manipulation_msgs.msg.MoveToJointPoseGoal(joints=self.motionDict[self.armAction], planningTime=20)
        self.armClient.send_goal(armGoal)
        self.armClient.wait_for_result(rospy.Duration.from_sec(30.0))
        armResult = self.armClient.get_result()

        if armResult is not None:
            if not armResult.success:
                rospy.loginfo('Joint Pose Planning failed!')
            else:
                rospy.loginfo('Joint Pose Planning Succeeded')
                self.jointPlanningSuccess = True
        else:
            rospy.loginfo('Time Out - No arm moving result received')




