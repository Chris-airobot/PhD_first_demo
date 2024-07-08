#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
from typing import List, Union
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from gpd_ros.msg import GraspConfig, GraspConfigList
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from robot_model import GEN3_LITE
from first_demo.srv import PCLFwdStatus
# from pathlib import Path
import numpy as np
import pyquaternion
from tf2_geometry_msgs import PoseStamped
import sensor_msgs.point_cloud2 as pc2
from camera import Camera
import tf.transformations as tf_trans

class FirstDemo:
    def __init__(self):
        rospy.init_node("demo")
        
        # Call the point cloud data service
        self.left_scan = rospy.ServiceProxy("/left_scan", AssembleScans2)
        self.right_scan = rospy.ServiceProxy("/right_scan", AssembleScans2)
        

        # Robot Object       
        self.robot = GEN3_LITE()
        self.robot.move_trajectories(self.robot.pre_grasp)
        self.grasp_process_timeout = rospy.Duration(5)
        
        # Publish the point cloud data into topic that GPD package which receives the input
        self.left_grasps_pub = rospy.Publisher("/left_cloud", PointCloud2, queue_size=1)
        self.right_grasps_pub = rospy.Publisher("/right_cloud", PointCloud2, queue_size=1)


        self.grasp_left_list: Union[List[GraspConfig], None] = None
        self.grasp_right_list: Union[List[GraspConfig], None] = None
        
        
        # Subscribe to the gpd topic and get the grasp gesture data, save it into the grasp_list through call_back function
        self.gpd_left_sub = rospy.Subscriber("/left_grasp/clustered_grasps", GraspConfigList, self.save_left_grasps)
        self.gpd_right_sub = rospy.Subscriber("/right_grasp/clustered_grasps", GraspConfigList, self.save_right_grasps)
        # Topic for visualization of the grasp pose
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)

        self.init_grasping_pose = None
        
        # Indicating which side to grasp
        self.left = True
        
        self.count = 0
        rospy.sleep(1)
        
        
        
    def scan_object(self) -> PointCloud2:
                
        '''
        Get the point cloud of the whole scene
        '''
        start_time = rospy.Time.now()
        rospy.sleep(2)
        end_time = rospy.Time.now()
        if self.left:
            stitch_response = self.left_scan(AssembleScans2Request(begin=start_time, end=end_time))
        else:
            stitch_response = self.right_scan(AssembleScans2Request(begin=start_time, end=end_time))
            
        stitched_cloud: PointCloud2 = stitch_response.cloud
        
        return stitched_cloud
    
    

    def save_left_grasps(self, data: GraspConfigList):
        self.grasp_left_list = data.grasps
    
    def save_right_grasps(self, data: GraspConfigList):
        self.grasp_right_list = data.grasps    
        
            

        
    def vector3ToNumpy(self, vec: Vector3) -> np.ndarray:
        return np.array([vec.x, vec.y, vec.z])        
        
        
        
    
    
    def find_left_grasps(self, start_time):
        while (self.grasp_left_list is None and 
               rospy.Time.now() - start_time < self.grasp_process_timeout):
            rospy.loginfo("waiting for left grasps")
            rospy.sleep(0.5)

            if self.grasp_left_list is None:
                rospy.logwarn("No grasps detected on pcl, restarting!!!")
                continue
        
        # Sort all grasps based on the gpd_ros's msg: GraspConfigList.grasps.score.data            
        self.grasp_left_list.sort(key=lambda x: x.score.data, reverse=True)
        grasp_performed = False
        for grasp in self.grasp_left_list:
            rot = np.zeros((3, 3))
            # grasp approach direction
            rot[:, 2] = self.vector3ToNumpy(grasp.approach)
            # hand closing direction
            rot[:, 0] = self.vector3ToNumpy(grasp.binormal)
            # hand axis
            rot[:, 1] = self.vector3ToNumpy(grasp.axis)
            
            
            # Turn the roll pitch yaw thing into the quaternion axis
            
            quat = pyquaternion.Quaternion(matrix=rot)
            pos: Point = grasp.position
            
            pos.z -= 0.03
            
            orientation = [180, 0, -90]
            
            
            
            # PointCloud offset position
            grasp_pose = Pose(
                position=pos,
                orientation=Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0]),
            )

            
            # Publish the grasp pose visualisation
            self.pose_pub.publish(
                PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
            )

            input("Hit Enter to execute")
            grasp_move_done = self.robot.move_pose([pos.x, pos.y, pos.z], orientation)
            
            if grasp_move_done:
                grasp_performed = True
                break




        if not grasp_performed:
            rospy.logwarn("No valid grasps planned to, restarting!!")
            return grasp_performed
                    
        # Grasped the object
        self.robot.move_gripper(1)
        rospy.sleep(1)
        return grasp_performed    
    
    
    
    def find_right_grasps(self, start_time):
        while (self.grasp_right_list is None and 
               rospy.Time.now() - start_time < self.grasp_process_timeout):
            rospy.loginfo("waiting for left grasps")
            rospy.sleep(0.5)

            if self.grasp_right_list is None:
                rospy.logwarn("No grasps detected on pcl, restarting!!!")
                continue
        
        # Sort all grasps based on the gpd_ros's msg: GraspConfigList.grasps.score.data            
        self.grasp_right_list.sort(key=lambda x: x.score.data, reverse=True)
        grasp_performed = False
        for grasp in self.grasp_right_list:
            rot = np.zeros((3, 3))
            # grasp approach direction
            rot[:, 2] = self.vector3ToNumpy(grasp.approach)
            # hand closing direction
            rot[:, 0] = self.vector3ToNumpy(grasp.binormal)
            # hand axis
            rot[:, 1] = self.vector3ToNumpy(grasp.axis)
            
            
            # Turn the roll pitch yaw thing into the quaternion axis
            
            quat = pyquaternion.Quaternion(matrix=rot)
            pos: Point = grasp.position
            
            pos.z -= 0.03
            
            orientation = [180, 0, -90]
            
            
            
            # PointCloud offset position
            grasp_pose = Pose(
                position=pos,
                orientation=Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0]),
            )

            
            # Publish the grasp pose visualisation
            self.pose_pub.publish(
                PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
            )

            input("Hit Enter to execute")
            grasp_move_done = self.robot.move_pose([pos.x, pos.y, pos.z], orientation)
            
            if grasp_move_done:
                grasp_performed = True
                break




        if not grasp_performed:
            rospy.logwarn("No valid grasps planned to, restarting!!")
            return grasp_performed
                    
        # Grasped the object
        self.robot.move_gripper(1)
        rospy.sleep(1)
        return grasp_performed
            
            
    
        
    def main(self):
        
        while not rospy.is_shutdown():
            self.robot.init_pose()
            self.robot.move_gripper(0)
            start_time = rospy.Time.now()
            
            # rospy.sleep(1)
            
            # Creates a bunch of grasping poses
            self.grasp_list = None
            
            # Get the point cloud data of the object
            pcd = self.scan_object()
            if self.left:
                self.left_grasps_pub.publish(pcd)
            else:
                self.right_grasps_pub.publish(pcd)
            
            # Just wait for the grasping, while the callback function is running at the back
            if self.left:
                print("Finding left grasp...")
                executed = self.find_left_grasps(start_time)
            else:
                executed = self.find_right_grasps(self.grasp_right_list, start_time)
            
            
            if not executed:
                rospy.logwarn("No valid grasps planned to, restarting!!")
                continue
                        
            # Grasped the object
            self.robot.move_gripper(1)
            rospy.sleep(2)
            
            
            
            
            


if __name__ == '__main__':
    # params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    
    try:
        grasper = FirstDemo()
        grasper.main()
        # grasper.robot.move_gripper(0.3)
        #[51 47 20]
    except KeyboardInterrupt:
        pass