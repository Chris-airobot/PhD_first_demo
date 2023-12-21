import numpy as np
from numpy import pi

def calculate_transform(a, d, alpha, theta):
    # Define the transformation matrix for a specific joint
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def calculate_jacobian(joint_angles):
    # Define DH parameters [a, d, alpha, theta]
    dh_params = np.array([
        [0,    0.2433, pi/2, joint_angles[0]],
        [0.28, 0.03,   pi,   joint_angles[1]+pi/2],
        [0,    0.02,   pi/2, joint_angles[2]+pi/2],
        [0,    0.245,  pi/2, joint_angles[3]+pi/2],
        [0,    0.057,  pi/2, joint_angles[4]+pi],
        [0,    0.235,  0,    joint_angles[5]+pi/2]
    ])

    # Initialize the Jacobian matrix
    J = np.zeros((6, 6))

    # Initialize the transformation matrix
    T = np.eye(4)
    z = [np.array([0, 0, 1])]
    p = [np.zeros(3)]

    for i in range(6):
        # Calculate the relative transformation between joint i and end-effector
        T_i = calculate_transform(*dh_params[i])        
        T = T @ T_i


        # Extract the rotational and translational components from the relative transformation
        z.append(T[:3, 2])
        p.append(T[:3, 3])

    # End-effector position
    p_end = p[-1]
    
    # Compute Jacobian columns
    for i in range(6):
        J[:3, i] = np.cross(z[i], p_end - p[i])
        J[3:, i] = z[i]


    return J
