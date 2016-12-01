# Installation:
1. Follow the setup instructions for [HLPR repositories](https://github.com/HLP-R/hlpr_documentation/wiki/Vector-Simulation-Setup-Steps). If you have the workspace setup, make sure you pull the latest version of `hlpr_manipulation`, `hlpr_simulator` and `rail_manipulation_msgs` (NOTE: there have been some recent changes!!).
2. Follow the setup instructions for [Pick and Place](https://github.com/gt-rail-internal/codebase/blob/master/weiyuliu/README.md).
3. Clone [robotiq_gripper_actions](https://github.com/GT-RAIL/robotiq_85_gripper_actions.git) in your workspace.
4. Clone the `hlpr_nri_demo` package in your workspace. This is in develop stage, so make sure you have the latest version.
5. To get segmented object transforms in map frame instead of base frame (useful for if you move your base after segmentation call), set `segmentation_frame_id: "map"` in the file `rail_segmentation/config/zones.yaml`.
6. For simulation (segmentation), set the point_cloud_topic in `rail_segmentation/src/Segmenter.cpp` to `/kinect/depth_registered/points`
7. For simulation (octomap), set the point_cloud_topic in `hlpr_manipulation/hlpr_wpi_jaco_moveit_config/config/sensors_kinect.yaml` to `/kinect/depth_registered/points`
8. For simulation, set `moveit_launch`, `wpi_jaco_launch`, `use_wpi_jaco_exec` to `true` in `hlpr_nri_demo/launch/vector_ycb.launch`. If you want manipulation to integrate kinect output, set `use_octomap` to `true` as well.
9. For the map, copy the two map files named `simulation_room` in the `map` folder to the folder `<catkin_ws_path>/src/vector_v1/vector_common/vector_navigation/vector_navigation_apps/maps/` and then edit the parameter `image` in `simulation_room.yaml` to `<catkin_ws_path>/src/vector_v1/vector_common/vector_navigation/vector_navigation_apps/maps/simulation_room.pgm`. Alternatively, you can create a new map by following the instructions in [Vector Navigation](https://github.com/StanleyInnovation/vector_v1/wiki/Navigation). 
10. Do `catkin_make` and source `devel/setup.bash`.


# Training Pick and Place (If required):
1. `roslaunch hlpr_nri_demo vector_ycb.launch`. (Note: before running this, comment/uncomment the objects you want to spawn in the file `hlpr_nri_demo/launch/spawn_ycb.launch`).
2. If you dont see all the objects on the table, delete the object model from gazebo and do `roslaunch hlpr_nri_demo spawn_ycb.launch`.
3. `roslaunch vector_remote_teleop vector_remote_teleop.launch`.
4. `rosrun rail_segmentation rail_segmentation`.
5. (Optional: to see object recognition output) `roslaunch rail_recognition rail_recognition_listener.launch`.
6. `roslaunch rail_pick_and_place_tools rail_pick_and_place_backend.launch`.
7. `roslaunch rail_pick_and_place_tools rail_pick_and_place_frontend.launch`.
8. Now train it on the objects you want to pick in the demo. Follow the instructions in Pick and Place tutorials for this.


# Launching the demo in simulation:
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

# Known Issues:
1. Sometimes the controller fails in executing a plan for no apparent reason. Might be a simulation thing only.
2. The octomap integration in simulator and `recognizedObjectsCallback` in `hlpr_moveit_wrapper` causes collision detection with the object and the table. Hence, the grasp fails if its near any of these two these. Look more into this. Temporary cure, switch off octomap and recognizeObjectCallback.
3. The object slips out of the gripper in simulation even if the grasp is on the centroid. This might not happen on the actual robot.
4. Error while calling IK. The number of joint positions and names in different in JointState message.
5. `grasp_frame` to `odom` lookup error

# Recently Solved Issues:
1. Time extrapolation error in `CommonActions::executePickup` while looking up tranform from `base_link` to `grasp_frame`. This happens because IK call fails since the transform for `grasp_frame` is not being published when IK service is called to move to `approachAnglePose`. Resolved by transforming the `approachAnglePose` back to `base_link` before IK call.
2. `HlprMoveitWrapper::moveToJointPose` did not work as expected in simulation. This is because the published `joint_states` in simulation are in a  different order than the actual robot. Fixed by explicitly indexing each Joint.
3. `HlpMoveitWrapper::moveToPose`, joint_names not consistent with the vector. Fixed, NOT PUSHED YET!
4. Arm motion planning failure is its too far from the object. Solved by adding another states which moves the base at a fixed distance to the object. WARNING: Table collision hasnt been taken care of here, but OCTOMAP might help? This can fail if the object is too far on the table.
5. Unable to find object if the base is too close. Solved by searching at a distance from the table (to increase the search area) and then moving closer once the object is found.
6. Changed `longest_valid_segment_fraction` from `0.05` to `0.02` in  `/hlpr_wpi_jaco_moveit_config/config/ompl_planning.yaml`. Reason: unknown :P
7. `hlpr_moveit_wrapper` sometimes prints "Timeout reached, stopping trajectory execution". This does not really effect anything, so dont worry about it. Solution, changed timeout in CartesionPathCallback from `1.5` to `5.0`.
8. `hlpr_moveit_wrapper/cartesian_path` throws the error "Lookup would require extrapolation into the past" when it plans a longer path (which may collide with table) to the object. This causes failure of "Motion Planning". Solution, added a tf.waitforTransform in `HlprMoveitWrapper::cartesianPathCallback`.
9. The `HlprMoveitWarpper::prepareGrasp` did not work as expected. That is, the allowed collision matrix did not really allow collision with the object to be grasped. Solution, gripper names in `hlpr_moveit_wrapper` were not consistent with vector. Added the prefix `right_`.

# Launching the demo on actual robot:
1. ssh in to the robot, run `roslaunch wpi_jaco_wrapper arm.launch home_arm_on_init:=true`
2. On the other machine, run `roslaunch hlpr_wpi_jaco_moveit_config hlpr_wpi_jaco_simple_moveit.launch`
3. `rosrun rail_segmentation rail_segmentation`.
4. `roslaunch rail_recognition object_recognition_listener.launch`.
5. `rosrun robotiq_85_gripper_actions gripper_actions`.
6. `rosrun hlpr_manipulation_actions hlpr_moveit_wrapper`.
7. `rosrun hlpr_manipulation_actions common_actions`.
8. `rosrun hlpr_manipulation_actions primitive_actions`.
9. `roslaunch vector_viz view_robot.launch function:=map_nav`.
10. `roslaunch vector_navigation_apps 2d_map_nav_demo.launch map_file:=simulation_room`.
11. Give a 2d pose estimate using RVIZ. The robot will localize by driving in a circle. (Your FSM later would get stuck while waiting for move_base action client if you dont do this)
12. `rosrun hlpr_nri_demo fsm_main.py`

# New Problems while porting to actual robot:
1. Octomap has a lot of noise. motion planning fails way too often!
2. Motion planning fails while moving to tuck position. 
3. Motion planning fails while moving to aprroachAnglePose. 
4. Time extrapolation error in CartesianPathCallback might be solved with ros::Time(0) instead of ros::Time::now. Needs to be checked.


