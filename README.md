# First Demo of the PhD

Packages used 
- aruco_ros: aruco detection
- easy_handeye: camera calibration
- realsense-ros: d405 camera package
- ros_kortex: kinova_gen3_lite robot package

Necessary modification for the Packages:
1. realsense-ros/realsense2_camera/include/constants.h:\n
   line 37, PID of the D405 camera
   It seems that they fixed the repo, but I am lazy to download it again

3. easy_handeye/easy_handeye/launch/calibrate.launch:\n
   Line 12, default is changed to "/kinova_gen3_lite", which is the catkin workspace of the folder
   Line 13, default is changed from "Manipulator" to "arm", the package is written for UR series robot, different name conventions 
