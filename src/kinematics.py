"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    pass


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    pass


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    pass


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    homo_matrix = np.eye(4)
    for i in range(len(s_lst)):
        w = s_lst[i][:3]
        v = s_lst[i][3:]
        s_matrix = to_s_matrix(w, v)
        exp = expm(s_matrix * joint_angles[i])
        homo_matrix = np.matmul(homo_matrix, exp)
        
    homo_matrix = np.matmul(homo_matrix, m_mat)

    return homo_matrix


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    s_matrix = np.zeros([4, 4])
    s_matrix[0:3, 0:3] = np.array([[0, -w[2], w[1]],
                                   [w[2], 0, -w[0]],
                                   [-w[1], w[0], 0]])
    s_matrix[0:3, 3] = v
    return s_matrix

def IK_geometric(pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    l1 = 0.10391
    l2 = 0.20573
    l3 = 0.20
    l4 = 0.17415
    # beta = np.pi - np.arccos((0.05**2 + l2**2 - 0.2**2) / (2 * 0.05 * l2))
    theta1 = np.arctan2(-pose[0], pose[1])
    x = np.sqrt(pose[0]**2 + pose[1]**2) 
    y = pose[2] - l1
    # print("pose[2]", pose[2])
    # if(pose[2] > l1):
    #     Y = pose[2] + l4 * np.sin(pose[3]) - l1
    # else:
    #     Y = -l4 * np.sin(pose[3]) - l1
    X = x - l4 * np.cos(pose[3])
    Y = y - l4 * np.sin(pose[3])
    # print("l4*sin(pose[3])",l4 * np.sin(pose[3]))
    # print("X: ", X)
    # print("Y: ", Y)
    alpha = np.arccos((X**2 + Y**2 - l2**2 - l3**2) / (2 * l2 * l3))
    # theta3 = beta - alpha
    theta3 = np.arccos(((np.square(X)+np.square(Y))-(np.square(l2)+np.square(l3)))/(2*l2*l3))
    # print("theta3", theta3)
    # gama1 = np.arctan2(Y - l1, X)
    gama1 = np.arctan2(Y, X)
    gama2 = np.arctan2(l3 * np.sin(alpha), l2 + l3 * np.cos(alpha))
    theta2 = np.pi/2 - gama1 - gama2
    # print("theta2", theta2)

    # theta4 = np.abs(theta3 + pose[3])
    # theta4 = -pose[3] + theta2 + theta3
    theta4 = np.pi/2 - pose[3] - theta2 - theta3 ##correct expression of theta4- by Surya
    # print("pose[3]", pose[3])
    # print("theta4", theta4)
    theta5 = 0 
    return np.array([theta1, theta2 - 0.245, theta3 - 1.325, theta4, theta5], dtype=np.float32)

        


# def IK_geometric(pose):
#     """!
#     @brief      Get all possible joint configs that produce the pose.

#                 TODO: Convert a desired end-effector pose vector as np.array to joint angles

#     @param      dh_params  The dh parameters
#     @param      pose       The desired pose vector as np.array 

#     @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
#                 configuration
#     """
#     try:
#         l = np.array([0.10391, 0.200, 0.05, 0.20, 0.17415, 0.20573])

#         theta1 = np.arctan2(-pose[0], pose[1])
#         # print("THETA1: ", theta1)
#         if(np.sqrt(pose[0]**2 + pose[1]**2) <= 0.35 and pose[2] <= 0.35):
#             theta2_offset = 0.055 #radians
#             theta3_offset = -0.09 #radians
#             theta4_offset = -0.19 #radians
#             # print("ORTHOGONAL")
#             X = np.sqrt(pose[0]**2 + pose[1]**2)
#             Y = pose[2] + l[4] - l[0]
#             # print("pose[0]: ", pose[0])
#             # print("pose[1]: ", pose[1])
#             # print("pose[2]: ", pose[2])
#             # print("X: ", X)
#             # print("Y: ", Y)
#             try:
#                 alpha = np.arccos((X**2 + Y**2 - l[3]**2 - l[5]**2) / (2 * l[3] * l[5]))
#             except ValueError:
#                 return "Error calculating alpha due to invalid acos argument."
#             # print("NUMER: ", X**2 + Y**2 - l[3]**2 - l[5]**2)
#             # print("DENOM: ", 2*l[3]*l[5])
#             a = np.arctan2(Y, X)
#             b = np.arctan2(l[3]*np.sin(alpha), l[5] + l[3]*np.cos(alpha))
#             c = np.arctan2(l[2], l[1])
#             d = np.arctan2(l[1], l[2]) 
#             print("a:", np.degrees(a))
#             print("b:", np.degrees(b))
#             print("c:", np.degrees(c))
#             print("d:", np.degrees(d))
#             print("alpha:", np.degrees(alpha))
#             theta2_ = a + b
            
#             theta2 = np.pi/2 - theta2_ - c
#             theta3 = np.pi - alpha - d
#             theta4 = -np.pi/2 + theta2_ + alpha 
#             if(theta1 >= 0 and theta1 <= np.pi/2):
#                 theta5 = theta1 
#             elif(theta1 > np.pi/2):
#                 theta5 = theta1 
#             elif(theta1 <= 0 and theta1 >= -np.pi/2):
#                 theta5 = theta1
#             else:
#                 theta5 = theta1
#         else:
#             theta2_offset = -np.pi/4 #radians
#             theta3_offset = -0.19 + np.pi/2 #radians
#             theta4_offset = np.pi/2 #radians
#             # print("HORIZONTAL")
#             # X = np.sqrt((pose[0] + l[4] * np.sin(theta1))**2 + (pose[1] + l[4] * np.cos(theta1)**2))
#             X = np.sqrt(pose[0]**2 + pose[1]**2)
#             Y = pose[2] - l[0]
#             # print("pose[0]: ", pose[0])
#             # print("pose[1]: ", pose[1])
#             # print("pose[2]: ", pose[2])
#             # print("X: ", X)
#             # print("Y: ", Y)
#             try:
#                 alpha = np.arccos((X**2 + Y**2 - l[3]**2 - l[5]**2) / (2 * l[3] * l[5]))
#             except ValueError:
#                 return "Error calculating alpha due to invalid acos argument."
#             # print("NUMER: ", X**2 + Y**2 - l[3]**2 - l[5]**2)
#             # print("DENOM: ", 2*l[3]*l[5])
#             theta2_ = np.arctan2(Y, X) + np.arctan2(l[3]*np.sin(alpha), l[5] + l[3]*np.cos(alpha))
#             theta2 = np.pi/2 - theta2_ - np.arctan2(l[2], l[1]) + theta2_offset
#             theta3 = alpha - np.arctan2(l[1], l[2]) + theta3_offset
#             theta4 = theta2_ + alpha - np.pi + theta4_offset
#             theta5 = 0
#         return np.array([theta1, theta2, theta3, theta4, theta5], dtype=np.float32)
#     except Exception as e:
#         return f"An error occurred: {str(e)}"  
