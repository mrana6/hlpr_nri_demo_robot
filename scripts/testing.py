#!/usr/bin/env python

import smach
from smach_ros import SimpleActionState

from modules.pick_place_modules import *
from modules.navigation_module import *
from navigation_states import *


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
        self.centroidGraspPose.header.frame_id = "base_link"

        self.centroidGraspPose.pose.position.x = self.receivedCentroid.x
        self.centroidGraspPose.pose.position.y = self.receivedCentroid.y
        self.centroidGraspPose.pose.position.z = self.receivedCentroid.z

        self.centroidGraspPose.pose.orientation.x = 0.70711
        self.centroidGraspPose.pose.orientation.y = 0.0
        self.centroidGraspPose.pose.orientation.z = -0.70711
        self.centroidGraspPose.pose.orientation.w = 0.0


        pickupGoal = rail_manipulation_msgs.msg.PickupGoal(pose=self.centroidGraspPose,
                                                           lift=True,
                                                           verify=False,
                                                           attachObject=False)
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
    rospy.init_node('testing_demo')
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
        smach.StateMachine.add('SearchObject', SearchObjectState(),
                               transitions={'succeeded': 'PickUpCentroid',
                                            'failed': 'end'},
                               remapping={'startCommandIn': 'startCommand',
                                          'objectNameIn': 'objectName',
                                          'objectOut': 'recognizedObject'})

        smach.StateMachine.add('PickUpCentroid', PickUpCentroidState(),
                               transitions={'succeeded': 'end',
                                            'failed': 'end'},
                               remapping={'objectIn': 'recognizedObject'})


    # Execute SMACH plan
    outcome = sm.execute()