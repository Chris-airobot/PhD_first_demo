#!/usr/bin/env python3
from typing import Dict, List
import pandas as pd
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Pose
from scipy.spatial.transform import Rotation as R
from ros_numpy import numpify
import yaml
import os
from tf import TransformListener
from pydantic import BaseModel, Field
from tf2_geometry_msgs import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R

# Get loc of package on computer
ROOT_PATH = os.path.dirname(__file__)

def pose_difference(p_current, p_desired):
    """Inputs are supposed to be in a list of 7 elements, first 3 
    are translational elements, last 4 are quaternion elements

    Args:
        p_current: current pose of the end_effector
        p_desired: desired pose of the end_effector
    """
    
    delta_translation = np.array(p_desired[:3]) - np.array(p_current[:3])
    
    # Rotation Difference calculation
    current_rotation = R.from_euler('xyz', p_current[3:]).as_quat()
    desired_rotation = R.from_euler('xyz', p_desired[3:]).as_quat()
    
    
    delta_rotation = R.from_quat(desired_rotation) * R.from_quat(current_rotation).inv()
    delta_rotation= delta_rotation.as_euler('xyz')
    
    
    # delta_angles = euler_from_quaternion(delta_rotation_quat)
    # rotation_angles = [0, 0, 0]
    
    # for i in [0,1,2]:
    #     rotation_angles[i] = delta_angles[i]*180/np.pi

    return np.concatenate((delta_translation, delta_rotation), axis= None)



def init_tf_tree() -> TransformListener:
    """Launch the tf2 transform tree to ensure transforms are available for use"""
    tf_listener = TransformListener()

    transforms_available = False
    while not transforms_available:
        try:
            tf_listener.waitForTransform(
                "/base_link", "/base_link", rospy.Time(), rospy.Duration(0.1)
            )
            transforms_available = True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo("Waiting for tf tree")

    return tf_listener

def transform_pose(input_pose, from_frame, to_frame):
    # Initialize a tf2 Buffer and TransformListener
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Wait for the transformation to become available
    rospy.sleep(1.0)

    # Create a PoseStamped message from the input pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = from_frame
    pose_stamped.pose = input_pose

    try:
        # Use the TransformListener to transform the pose to the target frame
        transformed_pose_stamped = buffer.transform(pose_stamped, to_frame)

        # Extract the transformed pose
        transformed_pose = transformed_pose_stamped.pose

        return transformed_pose

    except tf2_ros.TransformException as e:
        rospy.logwarn("Transform failed: {}".format(e))
        return None





# def pose_reframe(original_pose: PoseStamped, angle_degrees):
#     # Convert angle from degrees to radians
#     angle_radians = math.radians(angle_degrees)
    
#     original_euler = transformations.euler_from_quaternion((original_pose.pose.orientation.x,
#                                                             original_pose.pose.orientation.y,
#                                                             original_pose.pose.orientation.z,
#                                                             original_pose.pose.orientation.w))
    
#     roll, pitch, yaw = original_euler[0], original_euler[1], original_euler[2]
    
#     yaw += angle_radians
    
#     # Create a quaternion representing the rotation
#     quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)

#     # Create a Transform object
#     transform = tf.TransformerROS()
#     transform.setTransform(original_pose.header.frame_id, 
#                            "transformed_frame", 
#                            rospy.Time.now(), 
#                            original_pose.header.stamp, 
#                            original_pose.header.frame_id, 
#                            Point(original_pose.pose.position.x, original_pose.pose.position.y, original_pose.pose.position.z), 
#                            quaternion)

#     # Transform the original pose
#     transformed_pose = transform.transformPose("transformed_frame", original_pose)

#     return transformed_pose

class TFFixer:
    def __init__(self):
        rospy.init_node("tool_frame_camlink_frame_transformer")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.broadcast = tf2_ros.StaticTransformBroadcaster()

        self.timeout = rospy.Duration(5.0)



    def announce_transform(self, parent_frame, child_frame, published=False):
        mode = "Published" if published else "Read"
        rospy.loginfo(f"{mode} transform from {parent_frame} to {child_frame}!")

    def main(self):
        try:
            T_color_cam_msg: TransformStamped = self.tfBuffer.lookup_transform(
                "camera_color_frame",
                "camera_link",
                rospy.Time(),
                self.timeout,
            )
            T_color_cam = numpify(T_color_cam_msg.transform)
            self.announce_transform("camera_color_frame", "camera_link")

            T_optic_color_msg: TransformStamped = self.tfBuffer.lookup_transform(
                "camera_color_optical_frame",
                "camera_color_frame",
                rospy.Time(),
                self.timeout,
            )
            T_optic_color = numpify(T_optic_color_msg.transform)
            self.announce_transform("camera_color_optical_frame", "camera_color_frame")

            T_tool0_optic_msg: TransformStamped = self.tfBuffer.lookup_transform(
                "tool0",
                "gt_color_optical",
                rospy.Time(),
                self.timeout,
            )
            T_tool0_optic = numpify(T_tool0_optic_msg.transform)
            self.announce_transform("tool0", "gt_color_optical")

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise TimeoutError("Waited too long for a tf to be available!")

        # Compute transform
        T_tool0_cam = T_tool0_optic @ T_optic_color @ T_color_cam
        # R_tool0_cam_quart = Quaternion(matrix=T_tool0_cam)
        R_tool0_cam_quart = R.as_quat(R.from_matrix(T_tool0_cam[:3, :3]))

        # Publish transform
        t = TransformStamped()
        t.header.frame_id = "tool0"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "camera_link"
        t.transform.translation.x = T_tool0_cam[0, 3]
        t.transform.translation.y = T_tool0_cam[1, 3]
        t.transform.translation.z = T_tool0_cam[2, 3]

        t.transform.rotation.x = R_tool0_cam_quart[0]
        t.transform.rotation.y = R_tool0_cam_quart[1]
        t.transform.rotation.z = R_tool0_cam_quart[2]
        t.transform.rotation.w = R_tool0_cam_quart[3]

        self.broadcast.sendTransform(t)

        self.announce_transform("tool0", "camera_link", published=True)

        rospy.spin()


if __name__ == "__main__":
    try:
        worker = TFFixer()
        worker.main()
    except rospy.ROSInterruptException:
        pass
