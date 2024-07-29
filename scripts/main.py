#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
from typing import List, Union
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from gpd_ros.msg import GraspConfig, GraspConfigList
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2, Image
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
from cv_bridge import CvBridge, CvBridgeError
import cv2

class FirstDemo:
    def __init__(self):
        rospy.init_node("demo") 
        self.bridge = CvBridge()
        
        # Call the point cloud data service
        self.left_scan = rospy.ServiceProxy("/left_scan", AssembleScans2)
        self.right_scan = rospy.ServiceProxy("/right_scan", AssembleScans2)
        
        # # indicates whether there are objects on the top of the table
        # self.left_object = None
        # self.right_object = None

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
        
        # self.left_detection = rospy.Subscriber("/left_detection", Bool, self.left_detection_callback)
        # self.left_counter = 0
        
        # self.right_detection = rospy.Subscriber("/right_detection", Bool, self.right_detection_callback)
        # self.right_counter = 0
        
        
        # Topic for visualization of the grasp pose
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)

        self.init_grasping_pose = None
        
        # Indicating which side to grasp
        self.left_work = True
        
        self.count = 0
        rospy.sleep(1)
        
            
        
        
    def table_detection(self):
        left_image = rospy.wait_for_message('/left_camera/color/image_raw',Image)
        right_image = rospy.wait_for_message('/right_camera/color/image_raw',Image)
    
        left_color_image = self.bridge.imgmsg_to_cv2(left_image, "bgr8")
        right_color_image = self.bridge.imgmsg_to_cv2(right_image, "bgr8")
        def process_image(color_image, left):
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Define the HSV range for grey color (adjust as needed)
            lower_grey = np.array([0, 0, 50])
            upper_grey = np.array([180, 50, 200])

            # Create a mask for the grey table
            grey_mask = cv2.inRange(hsv_image, lower_grey, upper_grey)

            # Invert the grey mask to get the objects
            objects_mask = cv2.bitwise_not(grey_mask)

            # Create a mask to exclude the fixed base element (adjust as needed)
            exclude_mask = np.ones(objects_mask.shape, dtype=np.uint8) * 255
            height, width = objects_mask.shape[:2]
            
            if left:
                exclude_mask[:30, width-120:] = 0
            else:
                exclude_mask[:80, :130] = 0
                
            # # exclude_mask[:70, :80] = 0  # Left top corner
            # exclude_mask[:30, width-120:] = 0  # Right top corner
            
            # Apply the exclude mask to the objects mask
            objects_mask = cv2.bitwise_and(objects_mask, objects_mask, mask=exclude_mask)

            text = 'left' if left else 'right'
            # Check if there are any objects (non-zero pixels in the objects mask)
            if cv2.countNonZero(objects_mask) > 0:
                print(f"Objects detected on {text} the table!")
                return True
            else:
                print(f"No objects detected on {text} the table.")
                return False
        self.left_detection = process_image(left_color_image, True)
        self.right_detection = process_image(right_color_image, False)

        # working in left, and there are objects in the left
        if not self.left_detection :
            self.left_work = False
        if not self.right_detection:
            self.left_work = True 
        
    
        
    def scan_object(self) -> PointCloud2:
                
        '''
        Get the point cloud of the whole scene
        '''
        start_time = rospy.Time.now()
        rospy.sleep(2)
        end_time = rospy.Time.now()
        if self.left_work:
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
            
            pos.z -= 0.00
            
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
            res = input("Hit Enter to execute")
            
            if res == 'q':
                continue
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
            rospy.loginfo("waiting for right grasps")
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
            
            pos.z -= 0.01
            
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

            res = input("Hit Enter to execute")
            if res == 'q':
                return False
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
            self.table_detection()
            self.robot.grasp_prep(self.left_work)
            
            start_time = rospy.Time.now()
            
            # rospy.sleep(1)
            
            # Creates a bunch of grasping poses
            self.grasp_list = None
            # Get the point cloud data of the object
            pcd = self.scan_object()
            if self.left_work:
                self.left_grasps_pub.publish(pcd)
            else:
                self.right_grasps_pub.publish(pcd)
            
            # Just wait for the grasping, while the callback function is running at the back
            if self.left_work:
                executed = self.find_left_grasps(start_time)
            else:
                executed = self.find_right_grasps(start_time)
            
            
            if not executed:
                rospy.logwarn("No valid grasps planned to, restarting!!")
                continue
                        
            # Grasped the object
            self.robot.move_gripper(1)
            rospy.sleep(1)
            
            self.robot.grasp_prep(self.left_work)
            self.robot.demo_release(self.left_work)
            self.robot.move_gripper(0)
            rospy.sleep(1)
            


if __name__ == '__main__':
    # params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    
    try:
        grasper = FirstDemo()
        grasper.main()
        # grasper.robot.move_gripper(0.3)
        #[51 47 20]
    except KeyboardInterrupt:
        pass