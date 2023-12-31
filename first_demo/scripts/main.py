#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
from typing import List, Union
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from gpd_ros.msg import GraspConfig, GraspConfigList
from robot_initialization import RobotInitialization
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from robot_initialization import RobotInitialization
# from util.utils import transform_pose
from first_demo.srv import PCLFwdStatus, PCLFwdStatusRequest
from pathlib import Path
import numpy as np
import pyquaternion
from tf2_geometry_msgs import PoseStamped
from scipy.linalg import lstsq
from gpd_ros.msg import CloudIndexed
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Int64

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
        
        # Load the files about default movements
        # self.params = Config.parse_obj(load_yaml(params_path))
        # Robot Object       
        self.robot = RobotInitialization()
        self.grasp_process_timeout = rospy.Duration(10)
        # Load the joint states in config/preset_locs.yaml        
        # self.key_joint_states = load_yaml(Path(self.params.paths.root) / self.params.paths.key_locs)
    
        # # Load the joint states in configs/trajectories/single_view_joints.csv
        # self.scan_joints = load_joint_trajectory(
        #     Path(self.params.paths.root) / self.params.paths.trajectories.scene_scan
        # )
        
        
        # Publish the point cloud data into topic that GPD package which receives the input
        self.get_grasps_pub = rospy.Publisher(
            "/cloud_to_get_grasps_on", PointCloud2, queue_size=1
        )


        # self.get_grasps_pub_cloud_index = rospy.Publisher('/cloud_indexed', CloudIndexed, queue_size=1)


        self.grasp_list: Union[List[GraspConfig], None] = None
        
        # Subscribe to the gpd topic and get the grasp gesture data, save it into the grasp_list through call_back function
        self.gpd_sub = rospy.Subscriber(
            "/detect_grasps/clustered_grasps", GraspConfigList, self.save_grasps
        )
        # Topic for visualization of the grasp pose
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)


        self.temp = 0
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
        
        
    def point_cloud_processing(self, pcd):
        cloud = []
        
        for p in point_cloud2.read_points(pcd):
            cloud.append([p[0], p[1], p[2]])
            
        cloud = np.asarray(cloud)
        A = np.c_[cloud[:,0], cloud[:,1], np.ones(cloud.shape[0])]
        b = cloud[:,2]
        C, _, _, _ = lstsq(A, b)
        a, b, c, d = C[0], C[1], -1., C[2] # coefficients of the form: a*x + b*y + c*z + d = 0.
        dist = ((a*cloud[:,0] + b*cloud[:,1] + d) - cloud[:,2])**2
        err = dist.sum()
        idx = np.where(dist > 0.01)
        
        msg = CloudIndexed()
        header = Header()
        header.frame_id = "/base_link"
        header.stamp = rospy.Time.now()
        msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
        msg.cloud_sources.view_points.append(Point(0,0,0))
        for i in range(cloud.shape[0]):
            msg.cloud_sources.camera_source.append(Int64(0))
        for i in idx[0]:
            msg.indices.append(Int64(i))    
        s = input('Hit [ENTER] to publish')
        self.get_grasps_pub_cloud_index.publish(msg)
        print(f'Published cloud with', len(msg.indices), 'indices')
        rospy.sleep(2)
  
                
        
        
    def vector3ToNumpy(self, vec: Vector3) -> np.ndarray:
        return np.array([vec.x, vec.y, vec.z])        
        
        
    def main(self):
        while not rospy.is_shutdown():
            self.robot.init_pose()
            start_time = rospy.Time.now()
            
            rospy.sleep(1)
            
            # Creates a bunch of grasping poses
            self.grasp_list = None
            
            # Get the point cloud data of the object
            pcd = self.scan_object()

            # self.point_cloud_processing(pcd)
            self.get_grasps_pub.publish(pcd)
            
            
            # Just wait for the grasping, while the callback function is running at the back
            while (
                self.grasp_list is None
                and rospy.Time.now() - start_time < self.grasp_process_timeout
            ):
                rospy.loginfo("waiting for grasps")
                rospy.sleep(0.05)

            if self.grasp_list is None:
                rospy.logwarn("No grasps detected on pcl, restarting!!!")
                continue
            
            
            # Sort all grasps based on the gpd_ros's msg: GraspConfigList.grasps.score.data            
            self.grasp_list.sort(key=lambda x: x.score.data, reverse=True)

            grasp_performed = False
            for grasp in self.grasp_list:
                rot = np.zeros((3, 3))
                # grasp approach direction
                rot[:, 2] = self.vector3ToNumpy(grasp.approach)
                # hand closing direction
                rot[:, 0] = self.vector3ToNumpy(grasp.binormal)
                # hand axis
                rot[:, 1] = self.vector3ToNumpy(grasp.axis)
                
                # Rot shape:
                # [approach.x  binormal.x  axis.x
                #  approach.y  binormal.y  axis.y
                #  approach.z  binormal.z  axis.z]
                

            
                # Turn the roll pitch yaw thing into the quaternion axis
                
                quat = pyquaternion.Quaternion(matrix=rot)
                pos = grasp.position
                # PointCloud offset position
                
                grasp_pose = Pose(
                    position=pos,
                    orientation=Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0]),
                )

                # print(f'Before transformation: {grasp_pose}\n\n')
                
                # grasp_pose = transform_pose(grasp_pose, 'base_link', 'fake_link')
                # print(f'After transformation: {grasp_pose}')
                
                
                
                grasp_pose_stamped = PoseStamped(
                    pose=grasp_pose, header=Header(frame_id="base_link")
                )
                # Publish the grasp pose visualisation
                self.pose_pub.publish(
                    PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
                )
                
                
                rospy.loginfo(f"Pose is: \n {grasp_pose_stamped}")
                
                # self.pose_pub.publish(grasp.approach)
                grasp_move_done = self.robot.move(target=grasp_pose_stamped)
                
                if grasp_move_done:
                    grasp_performed = True
                    break

            if not grasp_performed:
                rospy.logwarn("No valid grasps planned to, restarting!!")
                continue
            
            # Grasped the object
            self.robot.move_gripper(0.1)
            self.robot.init_pose()
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