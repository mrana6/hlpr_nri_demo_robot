import roslib
import rospy
import smach
import smach_ros
import tf
from math import sqrt, pow

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import geometry_msgs.msg
import rail_manipulation_msgs.msg


class NavigationGoal(object):
    def __init__(self, withRobot=True):
        # Enable/disable listening and publishing to servers
        self.online = withRobot

        # Only implemented if withRobot is True
        # Create Client for vector_move_base action server
        if (self.online):
            self.move_base = actionlib.SimpleActionClient("vector_move_base", MoveBaseAction)
            rospy.loginfo("Waiting for move_base action server...")
            self.move_base.wait_for_server()

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation")
        rospy.loginfo("The end")

    def setGoalPose(self, pose):
        goal = MoveBaseGoal()
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'

        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal there
        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = pose.orientation.x
        goal.target_pose.pose.orientation.y = pose.orientation.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w

        print
        print "Navigation goal pose : " + str(goal.target_pose.pose.position.x) + ", " + str(
            goal.target_pose.pose.position.y) + ", " + str(goal.target_pose.pose.position.z) + " Orientation : " + str(
            goal.target_pose.pose.orientation.x) + ", " + str(goal.target_pose.pose.orientation.y) + ", " + str(
            goal.target_pose.pose.orientation.z) + ", " + str(goal.target_pose.pose.orientation.w)
        print

        if (self.online):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)

            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(120))

            # If we don't get there in time, abort the goal
            if not finished_within_time:
                rospy.loginfo("Timed out achieving goal")
                return False
            else:
                state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                return True
            else:
                return False
        else:
            return True



class BaseAdjuster:
    def __init__(self, distIn=0.3, objectIn= None ):
        self.receivedDist = distIn
        self.receivedObject = objectIn
        self.transformedObject = None
        self.objectCentroid = None
        self.objectDist = None
        self.navPose = geometry_msgs.msg.Pose()
        self.tfListener = tf.TransformListener()
        self.navGoal = NavigationGoal(True)
        self.adjustedFlag = False

    def adjustBase(self):
        if self.receivedObject is not None:
            self.objectCentroid = self.receivedObject.centroid
            self.objectDist = sqrt(pow(self.objectCentroid.x,2) + pow(self.objectCentroid.y,2) + pow(self.objectCentroid.z,2))

            #Check distance from object
            if self.objectDist > self.receivedDist:
                rospy.loginfo('Robot is too far. Moving Closer!')

                # Transform the grasps to map frame
                # Reason: extracted grasps are wrt base_link, moving base_link will spoil
                #         Hence, transforming to fixed map link

                #TODO: Uncomment this line if segmentation_zone is "base_link" instead of map
                #self.transformedObject = transformObject(self.receivedObject, 'map', 'base_link')
                self.transformedObject = self.receivedObject

                self.navPose.position.x = self.transformedObject.centroid.x - self.receivedDist
                self.navPose.position.y = self.transformedObject.centroid.y
                self.navPose.position.z = self.transformedObject.centroid.z

                self.navPose.orientation.x = 0.0
                self.navPose.orientation.y = 0.0
                self.navPose.orientation.z = 0.0
                self.navPose.orientation.w = 1.0


                # Listen the current base link tranform and move a bit forward
                # t = self.tfListener.getLatestCommonTime("/base_link", "/map")
                # position, quaternion = self.tfListener.lookupTransform("/map", "/base_link", t)
                # print position
                # print quaternion
                # self.navPose.position.x = position[0]+0.8  #TODO: This is inaccurate
                # self.navPose.position.y = position[1]
                # self.navPose.position.z = position[2]
                #
                # self.navPose.orientation.x = quaternion[0]
                # self.navPose.orientation.y = quaternion[1]
                # self.navPose.orientation.z = quaternion[2]
                # self.navPose.orientation.w = quaternion[3]

                self.adjustedFlag = self.navGoal.setGoalPose(self.navPose)

            else:
                self.adjustedFlag = True

        return self.adjustedFlag





# Trasfrorm Grasps to target frame (the grasp source frame is taken from header)
def transformGrasps(graspsIn, targetFrame = 'map'):
    transformedGrasps = []
    for grasp in graspsIn:
        transformedGrasp = grasp
        tfListener = tf.TransformListener()
        tfListener.waitForTransform(targetFrame, grasp.grasp_pose.header.frame_id, rospy.Time(), rospy.Duration(4.0))
        graspPose = tfListener.transformPose(targetFrame, grasp.grasp_pose)
        graspPose.header.frame_id = targetFrame
        transformedGrasp.grasp_pose = graspPose
        transformedGrasps.append(transformedGrasp)

    return transformedGrasps


# Trasform all the components in the object
def transformObject(objectIn, targetFrame = 'map', sourceFrame = 'base_link'):

    tfListener = tf.TransformListener()
    transformedObject = objectIn

    # Transforming grasps from source(default:base_link) to target frame (default:map)
    graspsIn = objectIn.grasps
    transformedGrasps = transformGrasps(graspsIn, targetFrame)
    transformedObject.grasps = transformedGrasps

    # Transforming centroid and center from grasp frame to target
    centroidStamped = geometry_msgs.msg.PointStamped()
    centroidStamped.header.frame_id = sourceFrame
    centroidStamped.point = objectIn.centroid

    centerStamped = geometry_msgs.msg.PointStamped()
    centerStamped.header.frame_id = sourceFrame
    centerStamped.point = objectIn.center


    tfListener.waitForTransform(targetFrame, sourceFrame, rospy.Time(), rospy.Duration(4.0))
    transformedCentroid = tfListener.transformPoint(targetFrame, centroidStamped)
    transformedCenter = tfListener.transformPoint(targetFrame, centerStamped)


    transformedObject.centroid = transformedCentroid.point
    transformedObject.center = transformedCenter.point


    return transformedObject














