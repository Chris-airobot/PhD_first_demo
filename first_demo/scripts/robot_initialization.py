import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
import rospy
import moveit_commander
from typing import Tuple, Union
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotTrajectory
from geometry_msgs.msg import Pose, Point,Quaternion, PoseStamped
from std_msgs.msg import Header
# from tf2_geometry_msgs import PoseStamped
PlanTuple = Tuple[bool, RobotTrajectory, float, MoveItErrorCodes]

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
        # self.init_pose()
        
        
    def move_gripper(self, value: float):
        '''
        Moves the gripper to the relative positons
        
        Args:
        value (float): a value from 0 to 1 to scale the gripper
        
        '''
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
        '''
        Adding a table under the robot to ensure it does not hit the table in reality
        '''
        table_size = [2, 2, 0.86]
        
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0  
        table_pose.pose.position.y = 0  
        table_pose.pose.position.z = -table_size[2]/2-0.040001 
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=table_size)

    
    
    
    
    def init_pose(self):
        '''
        Hard-coded home pose (for easy_handeye package) of the robot
        '''
        # home pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        # joint_values[0] = 0
        # joint_values[1] = -16 * pi / 180
        # joint_values[2] = 75 * pi / 180
        # joint_values[3] = 0
        # joint_values[4] = -60 * pi / 180
        # joint_values[5] = 0
        joint_values[0] = 0
        joint_values[1] = 0
        joint_values[2] = 0
        joint_values[3] = 0
        joint_values[4] = 0
        joint_values[5] = 0        
        self.arm_group.go(joint_values, wait=True)
        self.move_gripper(1)
        
    def get_cartesion_pose(self):
        '''
        Get the current pose and display it
        '''
        pose = self.arm_group.get_current_pose()
        
        print(f'The cartesian pose is:')
        print(pose.pose)
        
        return pose
    

    def get_arm_joint_values(self):
        '''
        Get the current joint and display it
        '''
        joints = self.arm_group.get_current_joint_values()
        
        for i in range(len(joints)):
            print(f"- joint_{i+1}: {joints[i]}")
            
        return joints
    
    
    def reach_joint_angles(self, values):
        '''
        vertical pose of the robot
        '''
        joint_values = self.arm_group.get_current_joint_values()
        joint_values = values
        self.arm_group.go(joint_values, wait=True)
        

    
    def camera_calibration_pose(self):
        '''
        Hard-coded calibration pose (for easy_handeye package) of the robot 
        '''

        # Gripper Pose
        # self.move_gripper(0.85)
        # Calibration pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[0] = -8 * pi/180
        joint_values[1] = -75 * pi / 180
        joint_values[2] = 74 * pi / 180
        joint_values[3] = 83 * pi / 180
        joint_values[4] = -67 * pi / 180
        joint_values[5] = -83 * pi / 180
        self.arm_group.go(joint_values, wait=True)
        
    def move(self, target):
        # self.arm_group.set_pose_target(target)
        # return True
        try:
            self.arm_group.set_joint_value_target(target, True)
            plan_tuple: PlanTuple = self.arm_group.plan()
            plan = self.unpack_plan(plan_tuple)
            input("Press to proceed")
            attempted = self.arm_group.execute(plan, wait=True)
            self.move_gripper(1)
        except:
            attempted = False
            
            
        return attempted
    
    
    def unpack_plan(self, plan_tuple: PlanTuple) -> Union[RobotTrajectory, None]:
        """Function used to unpack the tuple returned when planning with move_group.
        This seems to be different than is was in ros melodic, so this function
        is needed to adapt the old code to the changes.

        Args:
            plan_tuple: A plan tuple containing the plan and other success data.

        Returns:
            If the planning was successful, a trajectory that can be directly used for
            visualization and motion. If unsuccessful, None is returned.
        """

        # plan_tuple[0] is the success boolean flag
        if plan_tuple[0]:
            return plan_tuple[1]  # The RobotTrajectory
        else:
            # If needed, the exact error code can be parsed from plan_tuple[3]
            return None
    
    
    def check_poses(self):
        pos = Point
        pos.x = 0.5
        pos.y = 0.2
        pos.z = 0.1
        grasp_pose = Pose(
            position=pos,
            orientation=Quaternion(x=-0.5, y=0.5,  z=-0.5, w=0.5),
        )
        target = PoseStamped(
                    pose=grasp_pose, header=Header(frame_id="base_link")
        )
        
        self.arm_group.set_joint_value_target(target, True)
        plan_tuple: PlanTuple = self.arm_group.plan()
        plan = self.unpack_plan(plan_tuple)
        input("Press to proceed")
        attempted = self.arm_group.execute(plan, wait=True)
        # self.move_gripper(1)