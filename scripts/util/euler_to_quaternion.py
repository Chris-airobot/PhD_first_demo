#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from math import pi

'''
    This file is used for mannual turning axis of the roll, pitch, yaw and quaternion

'''


qw = 0.02048402177407003
qx =-0.6758548157988307
qy = -0.007358205134506801
qz =  0.7367133293423479
# def get_rotation (msg):
#     global roll, pitch, yaw
#     orientation_q = msg.pose.pose.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

rospy.init_node('my_quaternion_to_euler')
pose = Pose()
pose.orientation.x = qx
pose.orientation.y = qy
pose.orientation.z = qz
pose.orientation.w = qw


quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)

angles = euler_from_quaternion(quaternion)

roll, pitch, yaw = angles[0], angles[1], angles[2]
print(f'Roll is:{roll}, Pitch is: {pitch}, Yaw is:{yaw}')
# pitch = 0
# roll += pi  # red axis stay the same, beside the two, the first one
yaw += 3 * pi/180
quat = quaternion_from_euler (roll, pitch, yaw)
print(f"quat is: {quat} in x, y, z, w")


# sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

# r = rospy.Rate(1)
# while not rospy.is_shutdown():
#     quat = quaternion_from_euler (roll, pitch,yaw)

#     r.sleep()