#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from math import pi
import yaml
'''
    This file is used for mannual turning axis of the roll, pitch, yaw and quaternion

'''


f = open("/home/riot/kinova_gen3_lite/src/first_demo/config/kinova_gen3_lite_eye_on_base.yaml", "r")
for x in f:
    if x.startswith("  qw:"):
        qw = float(x[5:])
    elif x.startswith("  qx:"):
        qx = float(x[5:])
    elif x.startswith("  qy:"):
        qy = float(x[5:])
    elif x.startswith("  qz:"):
        qz = float(x[5:])
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
# pitch = 0   # blue axis stay the same, beside the two, the first one
# roll += pi  # red axis stay the same, beside the two, the first one


# This is in table plane rotation
# roll += 3 * pi/180  

# pitch -= 10 * pi/180

yaw += 5 * pi/180  
quat = quaternion_from_euler (roll, pitch, yaw)
print(f"quat is: {quat} in x, y, z, w")



# Open the YAML file and load its contents
with open('/home/riot/kinova_gen3_lite/src/first_demo/config/kinova_gen3_lite_eye_on_base.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Update the value of qw
data['transformation']['qw'] = float(quat[3])  # Change the value to whatever you need
data['transformation']['qx'] = float(quat[0])  # Change the value to whatever you need
data['transformation']['qy'] = float(quat[1])  # Change the value to whatever you need
data['transformation']['qz'] = float(quat[2])  # Change the value to whatever you need

# Write the updated content back to the file
with open('/home/riot/kinova_gen3_lite/src/first_demo/config/kinova_gen3_lite_eye_on_base.yaml', 'w') as file:
    yaml.dump(data, file)
