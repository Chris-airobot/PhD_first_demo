import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

class RobotInitialization:
    def __init__(self):
        
        # Initialize moveit! package
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Group names of the robot, found it by running the actual robot
        self.arm_group_name = 'arm'
        self.gripper_group_name = 'gripper'
        
        # Robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        # Interfaces for planning groups (group of joints) to plan an execute motions
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name, ns=rospy.get_namespace())

        # Ros publisher that is used to display trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace()+'/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        self.init_scene()
        self.init_pose()
        
        
    def move_gripper(self, value):
    
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        # We only need one joint name since the rest will respond to one of them
        gripper_joint_name = gripper_joint_names[0]
        
        # Corrspond to the actual joint in the robot
        gripper_joint = self.robot.get_joint(gripper_joint_name)
        
        # Get the bounds for calculating the gripper's desired position 
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        target = gripper_joint.move(value * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos)
        
        
        return target
    
    def init_scene(self):
        
        table_size = [2, 2, 0.87]
        
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0  
        table_pose.pose.position.y = 0  
        table_pose.pose.position.z = -table_size[2]/2 
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=table_size)

    
    
    
    
    def init_pose(self):
        # home pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[0] = 0
        joint_values[1] = -16 * pi / 180
        joint_values[2] = 75 * pi / 180
        joint_values[3] = 0
        joint_values[4] = -60 * pi / 180
        joint_values[5] = 0
        self.arm_group.go(joint_values, wait=True)
        
    def get_cartesion_pose(self):
        # Get the current pose and display it
        pose = self.arm_group.get_current_pose()
        
        print(f'The cartesian pose is:')
        print(pose.pose)
        
        return pose
    

    def get_arm_joint_values(self):
        # Get the current joint and display it
        joints = self.arm_group.get_current_joint_values()
        
        for i in range(len(joints)):
            print(f"- joint_{i+1}: {joints[i]}")
            
        return joints
    
    
    def reach_joint_angles(self, values):
        # vertical pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values = values
        self.arm_group.go(joint_values)
        
