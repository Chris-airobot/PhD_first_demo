#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Header

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, Point
from math import pi
from robot_initialization import RobotInitialization
import numpy as np
from tf.transformations import quaternion_from_matrix
import tf_conversions
from utils import pose_reframe


class FirstDemo:
    def __init__(self):
       pass
    




    
    


def main():
    rospy.init_node("first_test")
    
    

    
    
    
    pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)
    
    robot = RobotInitialization()
    robot.camera_calibration_pose()
    
    # pos = Point()
    # pos.x = 0.4
    # pos.y = 0
    # pos.z = 0.1
    # grasp_pose = Pose(
    #     position=pos,
    #     orientation=Quaternion(x=1, y=0,  z=0, w=0),
    # )
    # target = PoseStamped(
    #     pose=grasp_pose, header=Header(frame_id="base_link")
    # )
    

    # # print(f'Before transformation: {grasp_pose}\n\n')
                
    # grasp_pose = pose_reframe(target, 90)
    
    
    
    
    # # x=0, y=-1.000000e+00,  z=0, w=6.123234e-17
    
    # pose_pub.publish(
    #     PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
    # )
    
    # robot.move(target)   

if __name__ == '__main__':
    main()
    # joints = [-0.206673, -1.704190, -0.010491, -1.653301, -1.514554, 1.803519]
    
    # for joint in joints:
    #     print(joint * 180/pi)