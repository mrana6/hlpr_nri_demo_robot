#!/usr/bin/env python

import smach
from smach_ros import SimpleActionState

from modules.pick_place_modules import *
from modules.navigation_module import *
from navigation_states import *


class ClearSceneState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.counter = 0
        self.status = 'failed'
        rospy.wait_for_service('/hlpr_moveit_wrapper/detach_objects')
        rospy.wait_for_service('/clear_octomap')
        rospy.wait_for_service('/rail_segmentation/clear')
        self.detachObjectsService = rospy.ServiceProxy('/hlpr_moveit_wrapper/detach_objects', std_srvs.srv.Empty)
        self.clearOctomapService = rospy.ServiceProxy('/clear_octomap', std_srvs.srv.Empty)
        self.clearSegmentedObjectsService = rospy.ServiceProxy('/rail_segmentation/clear', std_srvs.srv.Empty)


    def execute(self, userdata):
        rospy.loginfo('Executing state ClearScene')
        try:
            self.detachObjectsService()
            rospy.loginfo('Detach Object Service Call Succeeded')
        except:
            rospy.loginfo('Detach Object Service Call Failed')

        try:
            self.clearSegmentedObjectsService()
            rospy.loginfo('Clear Segmented Objects Service Call Succeeded')
        except:
            rospy.loginfo('Clear Segmented Objects Service Call Failed')

        try:
            self.clearOctomapService()
            rospy.loginfo('Clear Octomap Service Call Succeeded')
            self.status = 'succeeded'
        except:
            rospy.loginfo('Clear Octomap Service Call Failed')

        return self.status


class MoveArmState(smach.State):
    def __init__(self, commandIn):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.counter = 0
        self.status = 'failed'
        self.armMover = ArmMover(commandIn)
        # self.armClient = actionlib.SimpleActionClient('hlpr_manipulation/common_actions/arm_action',
        #                                                  rail_manipulation_msgs.msg.ArmAction)
        # self.armClient.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveArm')


        self.armMover.executeArmMotion()
        if self.armMover.jointPlanningSuccess:
            self.status = 'succeeded'
        # armGoal = rail_manipulation_msgs.msg.ArmGoal(action=self.inputCommand)
        # self.armClient.send_goal(armGoal)
        # self.armClient.wait_for_result(rospy.Duration.from_sec(30.0))
        # armResult = self.armClient.get_result()
        #
        # if armResult:
        #     self.status = 'succeeded'

        return self.status



class SearchObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
                             input_keys=['objectNameIn'],
                             output_keys=['objectOut'])
        self.counter = 0
        self.status = 'failed'
        self.objectSearcher = ObjectSearcher()

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')
        self.objectSearcher.requestedObjectName = userdata.objectNameIn
        self.counter += 1

        #self.objectSearcher.getRecognizedObject()
        self.objectSearcher.findObject()
        recognizedObject = self.objectSearcher.recognizedObject

        if recognizedObject is not None:
            self.status = 'succeeded'
            userdata.objectOut = recognizedObject

        return self.status

class AdjustBaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['objectIn', 'distIn'],
                             output_keys=['objectOut'])

        self.counter = 0
        self.status = 'failed'
        self.baseAdjuster = BaseAdjuster()
        self.adjustedFlag = False

    def execute(self, userdata):
        rospy.loginfo('Executing state AdjustBase')
        self.baseAdjuster.receivedObject = userdata.objectIn
        self.baseAdjuster.receivedDist = userdata.distIn
        self.adjustedFlag = self.baseAdjuster.adjustBase()
        transformedObject = self.baseAdjuster.transformedObject

        if transformedObject is not None and self.adjustedFlag is True:
            self.status = 'succeeded'
            userdata.objectOut = transformedObject

        return self.status

#define state PickObject
class PickObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
                             input_keys=['objectIn'])
        self.counter = 0
        self.status = 'failed'
        self.objectPicker = ObjectPicker()

    def execute(self, userdata):
        rospy.loginfo('Executing state PickObject')
        self.objectPicker.receivedGrasps = userdata.objectIn.grasps
        self.counter += 1
        self.objectPicker.executePickUp()

        if self.objectPicker.pickUpSuccess:
            self.status = 'succeeded'

        return self.status


class PickUpCentroidState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
                             input_keys=['objectIn'])
        self.counter = 0
        self.status = 'failed'
        self.receivedObject = None
        self.receivedCentroid = None
        self.pickupClient = actionlib.SimpleActionClient('hlpr_manipulation/common_actions/pickup',
                                                         rail_manipulation_msgs.msg.PickupAction)
        self.pickupClient.wait_for_server()
        self.centroidGraspPose = geometry_msgs.msg.PoseStamped()


    def execute(self, userdata):
        rospy.loginfo('Executing Centroid Pickup')
        self.receivedObject = userdata.objectIn
        self.receivedCentroid = self.receivedObject.centroid
        self.centroidGraspPose.header.frame_id = "map"

        self.centroidGraspPose.pose.position.x = self.receivedCentroid.x
        self.centroidGraspPose.pose.position.y = self.receivedCentroid.y
        self.centroidGraspPose.pose.position.z = self.receivedCentroid.z

        self.centroidGraspPose.pose.orientation.x = 0.70711
        self.centroidGraspPose.pose.orientation.y = 0.0
        self.centroidGraspPose.pose.orientation.z = -0.70711
        self.centroidGraspPose.pose.orientation.w = 0.0


        pickupGoal = rail_manipulation_msgs.msg.PickupGoal(pose=self.centroidGraspPose,
                                                           lift=True,
                                                           verify=True,
                                                           attachObject=True)
        self.counter += 1
        self.pickupClient.send_goal(pickupGoal)
        self.pickupClient.wait_for_result(rospy.Duration.from_sec(30.0))
        pickupResult = self.pickupClient.get_result()

        if pickupResult is not None:
            if not pickupResult.executionSuccess:
                rospy.loginfo('Motion Planning Failed')
            else:
                rospy.loginfo('Motion Planning Succeeded')
                self.status = 'succeeded'
                self.pickupClient.wait_for_result(rospy.Duration.from_sec(20.0))
                pickupResult = self.pickupClient.get_result()
                if not pickupResult.success:
                    rospy.loginfo('Pick Up Failed')
                else:
                    rospy.loginfo('Pick Up Succeeded')
                    self.status = 'succeeded'
        else:
            rospy.loginfo('Timed Out - No pickup result received')

        return self.status






if __name__ == '__main__':
    rospy.init_node('hlpr_nri_demo_state_machine')
    # armMover = ArmMover('Ready')
    # armMover.executeArmMotion()



    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Set FSM user data
    sm.userdata.startCommand = True
    sm.userdata.objectName = "BANANA_ARTIFICIAL"
    sm.userdata.grasps = None
    sm.userdata.recognizedObject = None
    sm.userdata.transformedObject = None
    sm.userdata.navGoalStart = 'table'
    sm.userdata.navGoalFinal = 'person'
    sm.userdata.distBaseToObject = 0.6

    # Open the container
    with sm:
        #Add states to the container
        smach.StateMachine.add('ClearScene', ClearSceneState(),
                               transitions={'succeeded':'TuckArm',
                                            'failed':'end'})

        smach.StateMachine.add('TuckArm', MoveArmState('Tuck'),
                               transitions={'succeeded':'GoToTable',
                                            'failed':'end'})

        smach.StateMachine.add('GoToTable', GoToPointState(True),
                               transitions={'succeeded': 'SearchObject',
                                            'failed': 'end'},
                               remapping={'navGoalIn': 'navGoalStart'})

        smach.StateMachine.add('SearchObject', SearchObjectState(),
                               transitions={'succeeded': 'ReadyArm',
                                            'failed': 'end'},
                               remapping={'startCommandIn': 'startCommand',
                                          'objectNameIn': 'objectName',
                                          'objectOut': 'recognizedObject'})

        smach.StateMachine.add('ReadyArm', MoveArmState('Ready'),
                               transitions={'succeeded': 'AdjustBase',
                                            'failed': 'end'})

        smach.StateMachine.add('AdjustBase', AdjustBaseState(),
                               transitions={'succeeded': 'PickObject',
                                            'failed': 'end'},
                               remapping={'objectIn':'recognizedObject',
                                          'distIn':'distBaseToObject',
                                          'objectOut':'transformedObject'})

        smach.StateMachine.add('PickObject', PickUpCentroidState(),
                               transitions={'succeeded': 'GoToPerson',
                                            'failed': 'end'},
                               remapping={'objectIn': 'transformedObject'})

        smach.StateMachine.add('GoToPerson', GoToPointState(True),
                               transitions={'succeeded': 'HandOffArm',
                                            'failed': 'end'},
                               remapping={'navGoalIn': 'navGoalFinal'})

        smach.StateMachine.add('HandOffArm', MoveArmState('HandOff'),
                               transitions={'succeeded': 'end',
                                            'failed': 'end'})


    # Execute SMACH plan
    outcome = sm.execute()