# First Demo of the PhD

Packages used 
- aruco_ros: aruco detection
- easy_handeye: camera calibration
- realsense-ros: d405 camera package
- ros_kortex: kinova_gen3_lite robot package

Necessary modification for the Packages:
1. realsense-ros/realsense2_camera/include/constants.h:<br>
   line 37, PID of the D405 camera<br>
   It seems that they fixed the repo, but I am lazy to download it again<br>
   Well, I did it a few days after, since the yellow color of modifying the package drives me crazy.

3. easy_handeye/easy_handeye/launch/calibrate.launch:<br>
   Line 12, default is changed to "/kinova_gen3_lite", which is the catkin workspace of the folder<br>
   Line 13, default is changed from "Manipulator" to "arm", the package is written for UR series robot, different name conventions<br>
   Same here, I copy the whole file and make specific changes. 



# Based on Jason's Code, the package that I need:

- gpd_ros and gpd (basically the same type)


# Command for saving the pcd file:


- rosrun pcl_ros pointcloud_to_pcd input:=/cloud_to_get_grasps_on \_prefix:=/home/riot/kinova_gen3_lite/src/first_demo/pcds/


## Dec 1:
- Rule out the camera calibration as one of the potential problem for non correct grasping position  
- Also checked the "gpd" package in low-level, and realize the grasp pose published is in "base_link" frame  
- Switching from "tool_frame" to "end_effector_link" fixes the problem of the axis of ArUco makrer goes up
- Still unclear about the reason why the pose is not correct


## Dec 6:
- Finally fixed the problem of wrong graps pose, it is actually due to the different settings bettwen the kinova_gen3_lite robot's end effector's frame axis and the UR series robot's end effector's frame axis 
- Changed the Grasp Pose transformation


## Dec 13:
- Changes the structure of the code, also looks at the low-level control of the robot 
- Next step is thinking about controlling the speed and how to rule of the box so that GPD does not consider the box as the object to grasp


## Dec 17:
- Found the Jacobian of the robot, trying to test it with the real robot
- Also created the gazebo launch of the robot, however, so far it seems that it can only run with the MoveIt! 
- Found that the numpy package has to be the version of 1.20.3 otherwise the ros_numpy will have incompatible errors


## Dec 18:
- Now the robot can move with the low-level method, also the resolved-rate-control method logic has been implemented, but it has error
- Next step is fixing that error and making sure to implement the rest of functions similar to Moveit! but in low-level.


## Dec 20:
- A pile of objects, robot takes actions to have a better exploration feedback