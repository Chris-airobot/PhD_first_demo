#!/usr/bin/python3

import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from robot_initialization import RobotInitialization

class FirstDemo:
    def __init__(self):
       pass
    
    
    
    


def main():
    rospy.init_node("first_test")
    
    robot = RobotInitialization()
    print(f'Now for going to the Vertical Pose')
    input()
    vertical_pose_joints = [0,0,0,0,0,0]
    robot.reach_joint_angles(vertical_pose_joints)
    
    print(f'Now for going to the Init Pose')
    input()
    robot.init_pose()
    
    print(f'Move the gripper:')
    input()
    robot.move_gripper(0.1)
    print(f'Next')
    input()
    robot.move_gripper(0.5)
    print(f'Another')
    input()    
    robot.move_gripper(1)
    print(f'Final')
    input()
    robot.move_gripper(0.5)

if __name__ == '__main__':
    main()