#!/usr/bin/env python3
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test',
                anonymous=True)


# robot group names
arm_group_name = 'arm'
gripper_group_name = "gripper"



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())



display_trajectory_publisher = rospy.Publisher(rospy.get_namespace()+'/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)






table_size = [2, 2, 0.87]

table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "base_link"
table_pose.pose.position.x = 0  
table_pose.pose.position.y = 0  
table_pose.pose.position.z = -table_size[2]/2
table_name = "table"
scene.add_box(table_name, table_pose, size=table_size)



quit()

























# We can get the name of the reference frame for this robot:
planning_frame = arm_group.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = arm_group.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print()


pose = arm_group.get_current_pose()
print(pose)

# We can get the joint values from the group and adjust some of the values:
joint_goal = arm_group.get_current_joint_values()

joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = pi/2
joint_goal[3] = pi/4
joint_goal[4] = 0
joint_goal[5] = pi/2


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
arm_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
arm_group.stop()


gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
print(gripper_joint_names)
gripper_joint_name = gripper_joint_names[0]
gripper_joint = robot.get_joint(gripper_joint_name)
gripper_max_absolute_pos = gripper_joint.max_bound()
gripper_min_absolute_pos = gripper_joint.min_bound()
print(f'gripper_joint is:{gripper_joint.name()}')
print(f'max_bound is: {gripper_max_absolute_pos}')
print(f'min_bound is: {gripper_min_absolute_pos}')
target = 0.8 * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos
current = gripper_joint.value()
while current < 1 and current > 0:
    gripper_joint.move(current-0.1, True)
    current = gripper_joint.value()

    print(f'target is: {target}')
    print(f'current value is: {current}')
# val = gripper_joint.move(target, True)