#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
from typing import List, Union
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from gpd_ros.msg import GraspConfig, GraspConfigList, CloudIndexed
from robot_initialization import RobotInitialization
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, Int64
from robot_initialization import RobotInitialization
from first_demo.srv import PCLFwdStatus, PCLFwdStatusRequest
# from pathlib import Path
import numpy as np
import pyquaternion
from tf2_geometry_msgs import PoseStamped
import sensor_msgs.point_cloud2 as pc2

class FirstDemo:
    def __init__(self):
        rospy.init_node("demo")
    
        rospy.wait_for_service("/realsense_processing/set_pcd_fwd_status_server")
        self.set_pcd_fwding = rospy.ServiceProxy(
            "/realsense_processing/set_pcd_fwd_status_server", PCLFwdStatus
        )
        
        # Call the point cloud data service
        rospy.wait_for_service("/realsense_processing/stitch_pcd_service")
        self.stitch_pcds = rospy.ServiceProxy(
            "/realsense_processing/stitch_pcd_service", AssembleScans2
        )
        

        # Robot Object       
        self.robot = RobotInitialization()
        self.grasp_process_timeout = rospy.Duration(10)
        
        # Publish the point cloud data into topic that GPD package which receives the input
        self.get_grasps_pub = rospy.Publisher(
            "/cloud_to_get_grasps_on", PointCloud2, queue_size=1
        )



        self.grasp_list: Union[List[GraspConfig], None] = None
        
        # Subscribe to the gpd topic and get the grasp gesture data, save it into the grasp_list through call_back function
        self.gpd_sub = rospy.Subscriber(
            "/detect_grasps/clustered_grasps", GraspConfigList, self.save_grasps
        )
        # Topic for visualization of the grasp pose
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)

        self.init_grasping_pose = None
        
        # Indicating which side to grasp
        self.left = True
        
        self.count = 0
        rospy.sleep(1)
        
        
        
    def scan_object(self) -> PointCloud2:
                
        '''
        Scanning the object while moving and getting the point cloud data of the object
        
        return:
        stitched_cloud (PointCloud2): point cloud of the object
        '''
        
        start_time = rospy.Time.now()
        self.set_pcd_fwding(Bool(data=True))

        # for joint in self.scan_joints:
        #     self.robot.move(joint, motion_type=MotionType.joint)
        #     rospy.sleep(0.01)
        rospy.sleep(2)
        self.set_pcd_fwding(Bool(data=False))
        end_time = rospy.Time.now()

        stitch_response = self.stitch_pcds(AssembleScans2Request(begin=start_time, end=end_time))
        stitched_cloud: PointCloud2 = stitch_response.cloud
        return stitched_cloud

    def save_grasps(self, data: GraspConfigList):
        self.grasp_list = data.grasps
        
        
    def split_pcd(self, pcd, left):
        # Convert PointCloud2 to list of points
        point_list = list(pc2.read_points(pcd, field_names=("x", "y", "z"), skip_nans=True))
        median_y = np.median([point[1] for point in point_list])
        # Split based on y-axis
        bottom_half = [point for point in point_list if point[1] < median_y]
        top_half = [point for point in point_list if point[1] >= median_y]

        # Create new PointCloud2 messages
        left_pcd = pc2.create_cloud_xyz32(header=pcd.header, points=bottom_half)
        right_pcd = pc2.create_cloud_xyz32(header=pcd.header, points=top_half)
        
        return left_pcd if left else right_pcd
            

        
    def vector3ToNumpy(self, vec: Vector3) -> np.ndarray:
        return np.array([vec.x, vec.y, vec.z])        
        
        
    def main(self):
        
        while not rospy.is_shutdown():
            self.robot.init_pose()
            self.init_grasping_pose = self.robot.arm_group.get_current_joint_values()
            start_time = rospy.Time.now()
            
            rospy.sleep(1)
            
            # Creates a bunch of grasping poses
            self.grasp_list = None
            
            # Get the point cloud data of the object
            pcd = self.scan_object()

            # Split the PCD
            pcd = self.split_pcd(pcd, self.left)
            
        
            self.get_grasps_pub.publish(pcd)
            
            
            # Just wait for the grasping, while the callback function is running at the back
            while (
                self.grasp_list is None
                and rospy.Time.now() - start_time < self.grasp_process_timeout
            ):
                rospy.loginfo("waiting for grasps")
                rospy.sleep(0.5)

            if self.grasp_list is None:
                rospy.logwarn("No grasps detected on pcl, restarting!!!")
                continue
            
            
            # Sort all grasps based on the gpd_ros's msg: GraspConfigList.grasps.score.data            
            self.grasp_list.sort(key=lambda x: x.score.data, reverse=True)
            grasp_performed = False
            for grasp in self.grasp_list:
                moved = False
                rot = np.zeros((3, 3))
                # grasp approach direction
                rot[:, 2] = self.vector3ToNumpy(grasp.approach)
                # hand closing direction
                rot[:, 0] = self.vector3ToNumpy(grasp.binormal)
                # hand axis
                rot[:, 1] = self.vector3ToNumpy(grasp.axis)
                
                # (R = [approach binormal axis])
                # Rot shape:
                # [approach.x  binormal.x  axis.x
                #  approach.y  binormal.y  axis.y
                #  approach.z  binormal.z  axis.z]
                

            
                # Turn the roll pitch yaw thing into the quaternion axis
                
                quat = pyquaternion.Quaternion(matrix=rot)
                pos = grasp.position
                
                
                quat = [0.005311705479287762, 0.7192909056869605, -0.6941433696979334, 0.027520194136891465]
                
                # PointCloud offset position
                grasp_pose = Pose(
                    position=pos,
                    orientation=Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0]),
                )

                
                
                
                grasp_pose_stamped = PoseStamped(
                    pose=grasp_pose, header=Header(frame_id="base_link")
                )
                # Publish the grasp pose visualisation
                self.pose_pub.publish(
                    PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
                )
                # roll, pitch, yaw = euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])
                rospy.loginfo(f"Pose is: \n {grasp_pose_stamped}")
                # rospy.loginfo(f"And the angles are: \n roll: {roll}, \n pitch: {pitch}, \n yaw: {yaw}")
                
                # self.pose_pub.publish(grasp.approach)
                grasp_move_done = self.robot.move(target=grasp_pose_stamped)
                
                moved = self.init_grasping_pose != self.robot.arm_group.get_current_joint_values()
                
                if abs(self.init_grasping_pose[0]-self.robot.arm_group.get_current_joint_values()[0]) <= 0.001:
                    moved = False
                
                print(f'init_grasping_pose is :{self.init_grasping_pose}')
                print(f'current_joint_values is :{self.robot.arm_group.get_current_joint_values()}')
                
                
                if grasp_move_done and moved:
                    grasp_performed = True
                    break
                
            if not grasp_performed:
                self.count += 1
                if self.count <= 2:
                    rospy.logwarn("No valid grasps planned to, restarting!!")
                    self.count = 0
                    continue
                elif self.count > 2:
                    self.left = False
                        
                        
                        
            # Grasped the object
            self.robot.move_gripper(0.1)

            self.robot.demo_move(self.left)
            self.robot.move_gripper(1)
            
            
            
            
            
            rospy.sleep(0.1)


if __name__ == '__main__':
    # params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    
    try:
        grasper = FirstDemo()
        grasper.main()
        # grasper.robot.move_gripper(0.3)
        #[51 47 20]
    except KeyboardInterrupt:
        pass