# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 22:49:56 2016

@author: baris
"""
import smach
import rospy
import sys
import time
import tf
import smach_ros

from geometry_msgs.msg import *
from std_msgs.msg import *
#from hlpr_speech_synthesis import speech_synthesizer
import modules.navigation_module as NM


class GoToPointState(smach.State):
    """
    This states navigates the robot to the NavGoal
    navGoal can be either 'table' or 'person'
    """

    def __init__(self, withRobot=True):
        # Define State outcomes, input_keys and output_keys
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['navGoalIn'])
        self.counter = 0

        # Calling Navigation Goal Class
        self.navGoal = NM.NavigationGoal(withRobot)

        # Assign table pose wrt world frame?
        # The robot navigates to this pose to fetch the object
        self.table = Pose()

        self.table.position.x = -0.0683037543297 # 1.8
        self.table.position.y = -0.104  # 1.3
        self.table.position.z = 0

        self.table.orientation.x = 0.000
        self.table.orientation.y = 0.000
        self.table.orientation.z = 0.0  # 0.733
        self.table.orientation.w = 1.0  # 0.68

        # Assign person pose wrt world frame
        # The robot navigates to this pose to release the object
        self.other = Pose()
        self.other.position.x = -1.28926753998  # 2.41
        self.other.position.y = 0.904734373093  # 4.11
        self.other.position.z = 0.0

        self.other.orientation.x = 0.000
        self.other.orientation.y = 0.000
        self.other.orientation.z = -0.198  # 0.668
        self.other.orientation.w = 0.98  # 0.744

        # Save the poses in Pose Dictionary
        self.poseDict = {'table': self.table, 'person': self.other}

        #self.ss = speech_synthesizer.SpeechSynthesizer()

    def execute(self, userdata):
        """
        Pass the specified navGoal Pose to plan path
        """

        rospy.loginfo('Navigating to a point')
        #self.ss.say("Now I am navigating to the " + userdata.navGoalIn)
        self.navGoal.setGoalPose(self.poseDict[userdata.navGoalIn])
        time.sleep(0.5)

        return 'succeeded'




