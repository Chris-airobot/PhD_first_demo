#!/usr/bin/env python3
from typing import Dict, List
import pandas as pd
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from ros_numpy import numpify
import yaml
import os

from pydantic import BaseModel, Field

# Get loc of package on computer
ROOT_PATH = os.path.dirname(__file__)


class TrajectoryModel(BaseModel):
    scene_scan: str
    single_view: str


class PathModel(BaseModel):
    # Statically add the root location to the pydantic model
    root: str = Field(ROOT_PATH, const=True)
    key_locs: str
    trajectories: TrajectoryModel


class Config(BaseModel):
    paths: PathModel



def load_yaml(path):
    with open(path, "r") as f:
        output = yaml.safe_load(f)
    return output


def load_joint_trajectory(path: str) -> List[Dict[str, List[float]]]:
    df = pd.read_csv(path)
    joint_dicts = [{col: row[col] for col in df.columns} for _, row in df.iterrows()]
    return joint_dicts





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
