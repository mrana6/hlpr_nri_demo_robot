# Installation
## Setting up the workspace:
1. Follow the setup instructions for [HLPR repositories](https://github.com/HLP-R/hlpr_documentation/wiki/Vector-Simulation-Setup-Steps). If you have the workspace setup, make sure you pull the latest version of `hlpr_manipulation`, `hlpr_simulator` and `rail_manipulation_msgs` (NOTE: there have been some recent changes!!).
2. Follow the setup instructions for [Pick and Place](https://github.com/gt-rail-internal/codebase/blob/master/weiyuliu/README.md).
3. Clone [robotiq_gripper_actions](https://github.com/GT-RAIL/robotiq_85_gripper_actions.git) in your workspace.
4. To get segmented object transforms in map frame instead of base frame (useful for if you move your base after segmentation call), set `segmentation_frame_id: "map"` in the file `rail_segmentation/config/zones.yaml`. WARNING: Dont do this if you are not adjusting base after search state.
5. For the map, copy the two map files named `simulation_room` in the `map` folder to the folder `<catkin_ws_path>/src/vector_v1/vector_common/vector_navigation/vector_navigation_apps/maps/` and then edit the parameter `image` in `simulation_room.yaml` to `<catkin_ws_path>/src/vector_v1/vector_common/vector_navigation/vector_navigation_apps/maps/simulation_room.pgm`. Alternatively, you can create a new map by following the instructions in [Vector Navigation](https://github.com/StanleyInnovation/vector_v1/wiki/Navigation). 
6. For simulation, Clone the `hlpr_nri_demo` package in your workspace. This is in develop stage, so make sure you have the latest version.
7. For simulation (segmentation), set the point_cloud_topic in `rail_segmentation/src/Segmenter.cpp` to `/kinect/depth_registered/points`
8. For simulation (octomap), set the point_cloud_topic in `hlpr_manipulation/hlpr_wpi_jaco_moveit_config/config/sensors_kinect.yaml` to `/kinect/depth_registered/points`
9. For simulation, set `moveit_launch`, `wpi_jaco_launch`, `use_wpi_jaco_exec` to `true` in `hlpr_nri_demo/launch/vector_ycb.launch`. If you want manipulation to integrate kinect output, set `use_octomap` to `true` as well.
10. For actual robot, clone the `hlpr_nri_demo_robot` package in your workspace. This is in develop stage, so make sure you have the latest version.
11. For actual robot (segmentation), set the point_cloud_topic in `rail_segmentation/src/Segmenter.cpp` to `/kinect/qhd/points`
12. For actual robot (octomap), set the point_cloud_topic in `hlpr_manipulation/hlpr_wpi_jaco_moveit_config/config/sensors_kinect.yaml` to `/kinect/qhd/points`
13. For acutal robot (correction), setup [HLPR Speech](https://github.com/HLP-R/hlpr_speech/wiki/Getting%20Started)
14. For actual robot (correction), setup [HLPR Kinesthetic Teaching](https://github.com/HLP-R/hlpr_kinesthetic_teaching/wiki)
10. Do `catkin_make` and source `devel/setup.bash`.

## Training Pick and Place (If required):
1. For simulation, `roslaunch hlpr_nri_demo vector_ycb.launch`. (Note: before running this, comment/uncomment the objects you want to spawn in the file `hlpr_nri_demo/launch/spawn_ycb.launch`).
2. For simulation, If you dont see all the objects on the table, delete the object model from gazebo and do `roslaunch hlpr_nri_demo spawn_ycb.launch`.
3. `roslaunch vector_remote_teleop vector_remote_teleop.launch`.
4. `rosrun rail_segmentation rail_segmentation`.
5. (Optional: to see object recognition output) `roslaunch rail_recognition rail_recognition_listener.launch`.
6. `roslaunch rail_pick_and_place_tools rail_pick_and_place_backend.launch`.
7. `roslaunch rail_pick_and_place_tools rail_pick_and_place_frontend.launch`.
8. Now train it on the objects you want to pick in the demo. Follow the instructions in Pick and Place tutorials for this.


# Simulation
## Launching the demo in simulation:
1. `roslaunch hlpr_nri_demo vector_ycb.launch`.
2. If you dont see all the objects on the table, delete the object model from gazebo and do `roslaunch hlpr_nri_demo spawn_ycb.launch`.
3. `rosrun rail_segmentation rail_segmentation`.
4. `roslaunch rail_recognition object_recognition_listener.launch`.
5. `roslaunch hlpr_nri_demo gripper_actions_launcher.launch`.
6. `rosrun hlpr_manipulation_actions hlpr_moveit_wrapper`.
7. `rosrun hlpr_manipulation_actions common_actions`.
8. `rosrun hlpr_manipulation_actions primitive_actions`.
9. `roslaunch vector_viz view_robot.launch function:=map_nav`.
10. `roslaunch vector_navigation_apps 2d_map_nav_demo.launch map_file:=simulation_room sim:=true`.
11. Give a 2d pose estimate using RVIZ. The robot will localize by driving in a circle. (Your FSM later would get stuck while waiting for move_base action client if you dont do this)
12. `rosrun hlpr_nri_demo fsm_main.py`

## Known Issues in Simulation:
1. Sometimes the controller fails in executing a plan for no apparent reason. Might be a simulation thing only.
2. The octomap integration in simulator and `recognizedObjectsCallback` in `hlpr_moveit_wrapper` causes collision detection with the object and the table. Hence, the grasp fails if its near any of these two these. Look more into this. Temporary cure, switch off octomap and recognizeObjectCallback.
3. The object slips out of the gripper in simulation even if the grasp is on the centroid. This does not happen on the actual robot.
4. Error while calling IK. The number of joint positions and names in different in JointState message. Does not effect anything.
5. `grasp_frame` to `odom` lookup error. Does not effect anything.

## Recently Solved Issues in simulation:
1. Time extrapolation error in `CommonActions::executePickup` while looking up tranform from `base_link` to `grasp_frame`. This happens because IK call fails since the transform for `grasp_frame` is not being published when IK service is called to move to `approachAnglePose`. Resolved by transforming the `approachAnglePose` back to `base_link` before IK call.
2. `HlprMoveitWrapper::moveToJointPose` did not work as expected in simulation. This is because the published `joint_states` in simulation are in a  different order than the actual robot. Fixed by explicitly indexing each Joint.
3. `HlpMoveitWrapper::moveToPose`, joint_names not consistent with the vector. Fixed, NOT PUSHED YET!
4. Arm motion planning failure is its too far from the object. Solved by adding another states which moves the base at a fixed distance to the object. WARNING: Table collision hasnt been taken care of here, but OCTOMAP might help? This can fail if the object is too far on the table.
5. Unable to find object if the base is too close. Solved by searching at a distance from the table (to increase the search area) and then moving closer once the object is found.
6. Changed `longest_valid_segment_fraction` from `0.05` to `0.02` in  `/hlpr_wpi_jaco_moveit_config/config/ompl_planning.yaml`. Reason: unknown :P
7. `hlpr_moveit_wrapper` sometimes prints "Timeout reached, stopping trajectory execution". This does not really effect anything, so dont worry about it. Solution, changed timeout in CartesionPathCallback from `1.5` to `5.0`.
8. `hlpr_moveit_wrapper/cartesian_path` throws the error "Lookup would require extrapolation into the past" when it plans a longer path (which may collide with table) to the object. This causes failure of "Motion Planning". Solution, added a tf.waitforTransform in `HlprMoveitWrapper::cartesianPathCallback`.
9. The `HlprMoveitWarpper::prepareGrasp` did not work as expected. That is, the allowed collision matrix did not really allow collision with the object to be grasped. Solution, gripper names in `hlpr_moveit_wrapper` were not consistent with vector. Added the prefix `right_`.


# Actual Robot
## Launching the demo on actual robot:
1. ssh in to the robot, run `roslaunch wpi_jaco_wrapper arm.launch`
2. On the local machine, run `roslaunch hlpr_wpi_jaco_moveit_config hlpr_wpi_jaco_simple_moveit.launch`
3. `roslaunch vector_remote_teleop vector_remote_teleop.launch`
4. For navigation, `roslaunch vector_viz view_robot.launch function:=map_nav`
5. For navigation, `roslaunch vector_navigation_apps 2d_map_nav_demo.launch map_file:=rail_lab3` (give a 2D pose estimate to localize)
6. For pick_place and hlpr_manipulation, `roslaunch hlpr_nri_demo_robot demo_bringup.launch`
7. For GC mode, `roslaunch hlpr_speech_recognition speech_rec.launch`
8. For GC mode, `roslaunch hlpr_kinesthetic_interaction basic_kinesthetic_interaction.launch`
9. For pick_place training, `roslaunch rail_pick_and_place_tools rail_pick_and_place_backend.launch`
10. For pick_place training, `roslaunch rail_pick_and_place_tools rail_pick_and_place_frontend.launch`
11. `rosrun hlpr_nri_demo fsm_with_navigation.py` (Make sure to set the correct table and person pose in navigation_states.py)

## Known Issues on Actual Robot: 
(NOTE: Simulation issues apply to actual robot as well)

1. Motion Planning fails if the object is not placed in the right place. This might be just due to arm structure itself.
2. CommonAction sometimes outputs exceeded maximum threshold for execution. Does not seem to effect anything.
3. Recgnizes the object at random places in the presence of noise. Might be fixed by defining a search boundary.
4. Navigation is jerky. Getting rid of it for now.
4. Training should not be dependent on the user interface.
5. Add speech 
6. Train robustly on a bunch of other objects. New objects.


## Recently Solved Issues on Actual Robot:
1. Octomap has a lot of noise. motion planning fails way too often. Fix, switched octomap topic to `/kinect/qhd/points instead` of `/kinect/sd/points` 
2. Motion planning fails while moving to tuck position. Fix, upperTuck position works fine.
3. Motion planning fails while moving to aprroachAnglePose. Less noise now so probably works.
4. Time extrapolation error in CartesianPathCallback might be solved with ros::Time(0) instead of ros::Time::now. Needs to be checked.
5. Motion is very slow sometimes even when the plan is there. Increase time out?
6. Centroid pick up is a bit off
7. RGB and depth for `kinect/qhd/points` and `kinect/hd/points` are offset, need calibration!. Update: Calibration done. calib_path was not setup correctly. Changed the calib_path in vector2.launch in vector1 to point to the correct folder while looking for files on vector2. Calibration has also improved segmentation results. TODO: upload the calibration parameters somewhere on the repo.
8. PointCloud and URDF offset. Update: Found that the pan_base_link transform wrt base_chassis_link was not correct. Changed $KINECT_PAN_TILT_XYZ in vector_config/vector_config.sh after trail and error. Current parameter values: KINECT_PAN_TILT_XYZ="0.17888476 0 1.00505"
KINECT_PAN_TILT_RPY="0 0 0"

9. The robot would collide with the object while moving to the approachAnglePose. Fixed by moving the approachAnglePose further up (changes -0.05 to -0.1 in CommonActions::executePickup)


