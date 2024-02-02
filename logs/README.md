# Log

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
- Find a new place of the camera position
- Use robotics-toolbox-python 
- Read the paper about the idea above

## Jan 29:
- Mainly focused on reading papers now
- Updated the orientation method, i.e. fixed orientation now
- Successfully split the point cloud for each half
- Next step is doing the continuously for left side and right side