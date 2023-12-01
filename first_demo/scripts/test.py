#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import rospy
import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, Vector3
from math import pi
from robot_initialization import RobotInitialization
import numpy as np
from tf.transformations import quaternion_from_matrix
import tf_conversions
class FirstDemo:
    def __init__(self):
       pass
    
    
    
    


def main():
    rospy.init_node("first_test")
    
    robot = RobotInitialization()
    robot.camera_calibration_pose()
    # robot.move_gripper(1)
    # print(f'Now for going to the Vertical Pose')
    # input()
    # vertical_pose_joints = [0,0,0,0,0,0]
    # robot.reach_joint_angles(vertical_pose_joints)
    
    # print(f'Now for going to the Init Pose')
    # input()
    # robot.init_pose()
    
    # print(f'Move the gripper:')
    # input()
    # robot.move_gripper(0.8)
    # print(f'Next')
    # input()
    # robot.move_gripper(0.5)
    # print(f'Another')
    # input()    
    # robot.move_gripper(1)
    # print(f'Final')
    # input()
    # robot.move_gripper(0.5)

if __name__ == '__main__':
    main()
    # joints = [-0.206673, -1.704190, -0.010491, -1.653301, -1.514554, 1.803519]
    
    # for joint in joints:
    #     print(joint * 180/pi)