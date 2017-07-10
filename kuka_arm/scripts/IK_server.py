#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import symbols, cos, sin, asin, acos, pi, sqrt, atan2, Abs, simplify, pprint, solveset, S
from sympy.matrices import Matrix, Transpose


# DH Parameters
s = {alpha0: 0,     a0: 0,      d1: 0.75,  q1: q1,           # 1
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: -pi/2 + q2,   # 2
     alpha2: 0,     a2: 1.25,   d3: 0,     q3: q3,           # 3
     alpha3: -pi/2, a3: -0.054, d4: 1.5,   q4: q4,           # 4
     alpha4: pi/2,  a4: 0,      d5: 0,     q5: q5,           # 5
     alpha5: -pi/2, a5: 0,      d6: 0,     q6: q6,           # 6
     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}            # G


# Rotation Functions
def rot_x(q):
    T = Matrix([[1, 0, 0, 0],
                [0, cos(q), -sin(q), 0],
                [0, sin(q), cos(q), 0],
                [0, 0, 0, 1]])
    return T

def trans_x(d):
    T = Matrix([[1, 0, 0, d],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
    return T

def rot_z(q):
    T = Matrix([[cos(q), -sin(q), 0, 0],
                [sin(q), cos(q), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
    return T


def trans_z(d):
    T = Matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, d],
                [0, 0, 0, 1]])
    return T

def rot_y(q):
    T = Matrix([[cos(q),  0, sin(q), 0],
                [0,       1,      0, 0],
                [-sin(q), 0, cos(q), 0],
                [0,       0,      0, 1]])
    return T



def dh_trans(alpha0, a0, d1, q1):
    T = rot_x(alpha0) * trans_x(a0) * rot_z(q1) * trans_z(d1)
    return T


T0_1 = dh_trans(alpha0, a0, d1, q1)
T0_1 = T0_1.subs(s)

T1_2 = dh_trans(alpha1, a1, d2, q2)
T1_2 = T1_2.subs(s)

T2_3 = dh_trans(alpha2, a2, d3, q3)
T2_3 = T2_3.subs(s)

T3_4 = dh_trans(alpha3, a3, d4, q4)
T3_4 = T3_4.subs(s)

T4_5 = dh_trans(alpha4, a4, d5, q5)
T4_5 = T4_5.subs(s)

T5_6 = dh_trans(alpha5, a5, d6, q6)
T5_6 = T5_6.subs(s)

T6_G = dh_trans(alpha6, a6, d7, q7)
T6_G = T6_G.subs(s)


# Combine Transforms

T0_2 = T0_1 * T1_2
T0_3 = T0_2 * T2_3
T0_4 = T0_3 * T3_4
T0_5 = T0_4 * T4_5
T0_6 = T0_5 * T5_6
T0_G = T0_6 * T6_G


# Corrections for URDF/Gazebo reference frames
R2_corr = rot_z(pi/2) * rot_x(pi/2)
R3_corr = R2_corr
R4_corr = rot_z(pi) * rot_y(-pi/2)
R5_corr = R2_corr
R6_corr = R4_corr
RG_corr = R4_corr

# Apply corrections (so we could compare with ROS output)
T0_2_corr = T0_2 * R2_corr
T0_3_corr = T0_3 * R3_corr
T0_4_corr = T0_4 * R4_corr
T0_5_corr = T0_5 * R5_corr
T0_6_corr = T0_6 * R6_corr
T0_G_corr = T0_G * RG_corr

# Define helper functions
def forward_kinematics(precision = 6, subs = {}):
    return T0_G.evalf(precision, subs=subs)

def grip_corr(m):
    r = m * RG_corr
    return r.evalf()

def get_rot_mat(r, p, y):
    Rxyz =  rot_z(y) * rot_y(p) * rot_x(r)
    return Rxyz

def get_pos_rot_mat(px, py, pz, roll, pitch, yaw):
    R = get_rot_mat(roll, pitch, yaw)
    R[0,3] = px
    R[1,3] = py
    R[2,3] = pz
    return R


def inverse_kinematics(O, s):

    # Position of EE that we should reach
    px = O[0,3]
    py = O[1,3]
    pz = O[2,3]

    # Get final rotation matrix
    R = O[0:3, 0:3]  # get_rot_mat(roll, pitch, yaw)
    # print('R =')
    # pprint(R)

    # Distance from wrist center to end-effector
    d = s[d7]

    # Wrist Center
    xc = px - d * R[0, 2]
    yc = py - d * R[1, 2]
    zc = pz - d * R[2, 2]

    # print('xc =', xc)
    # print('yc =', yc)
    # print('zc =', zc)

    # theta_1
    theta_1 = atan2(yc, xc)
    # print('theta_1 =', theta_1.evalf())

    # joint_2 center
    x0 = s[a1]
    z0 = s[d1]
    # print('x0 =', x0, ', z0 =', z0)

    # Link
    l1 = s[a2]
    # print('s[a3] = ', s[a3], ', s[d4] =', s[d4])
    l2 = sqrt(s[a3]**2 + s[d4]**2)
    theta30 = atan2(Abs(s[a3]), s[d4])
    # print('l1 =', l1, ', l2 =', l2)
    # print('theta30 =', theta30)

    # Dist from {0} to {wc}
    dwc = sqrt((sqrt(xc**2+yc**2)-x0)**2 + (zc-z0)**2)
    # print('dwc =', dwc)

    # Check validity
    assert(l1 + l2 > dwc)

    # Cosine law for cos(pi - beta):
    D = (l1**2 + l2**2 - dwc**2) / (2 * l1 * l2)
    # theta_3 = pi/2 - theta30 - atan2(sqrt(1 - D**2), D) # here is possible a second solution when elbow-down with "-" sign
    # theta_3_alt = pi/2 - theta30 - atan2(-sqrt(1 - D**2), D) # alt solution
    theta_3 = pi/2 - theta30 - acos(D) # here is possible a second solution when elbow-down with "-" sign
    theta_3_alt = pi/2 - theta30 + acos(D) # alt solution
    # print('D =', D)
    # print('theta_3 =', theta_3.evalf())
    # print('theta_3_alt =', theta_3_alt.evalf())

    # angle to l2 from l1
    bet = pi/2 + theta_3 + theta30
    # print('bet =', bet)

    # Find angle to {0} - {wc} line
    gam = atan2(sqrt(xc**2+yc**2) - x0, zc - z0)
    # print('gam =', gam)

    # Find angle between {0} - {wc} /_ l1
    # cos (alph) = E
    E = (l1**2 + dwc**2 - l2**2) / (2 * l1 * dwc)
    alph = acos(E)
    # print('E =', E)
    # print('alph =', alph)

    # if 0 < beta < pi then theta_2 = gam - alph else theta_2 = gam + alph
    if 0 < bet and bet < pi:
        theta_2 = gam - alph
    else:
        theta_2 = gam + alph
    # print('theta_2 =', theta_2)

    # pitch2 = theta_2 + theta_3
    # print('pitch2 =', pitch2.evalf())

    # Check by feeding angles to forward kinematics
    t04 = T0_4.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3, q4: 0})
    # print('wc_calc =')
    # pprint(t04)

    # Wrist center should be equal to what we've started from
    # print('xc_diff =', xc - t04[0,3])
    # print('yc_diff =', yc - t04[1,3])
    # print('zc_diff =', zc - t04[2,3])
    assert(abs(xc - t04[0,3]) < 1e-6)
    assert(abs(yc - t04[1,3]) < 1e-6)
    assert(abs(zc - t04[2,3]) < 1e-6)

    # Get R0_4 with theta_4 = 0
    R0_4 = t04[0:3, 0:3]
    # print('R0_4 =')
    # pprint(R0_4.evalf(6))

    # Calc R4_G - wrist rotation matrix
    R4_G = Transpose(R0_4) * R[0:3, 0:3]
    # print('R4_G =')
    # pprint(R4_G.evalf(6))

    # Calc symbolicaly wrist rotation matrix
    R4_G_sym = rot_z(q4) * rot_x(pi/2) * rot_z(q5) * rot_x(-pi/2) * rot_z(q6)


    # Show formula (for derivation)
    print('R4_G_sym =')
    pprint(simplify(R4_G_sym)) # just for visualization


    # Corner case q5 = 0
    print('R4_G_sym (q5=0) =')
    pprint(simplify(R4_G_sym.subs({q5: 0}))) # just for visualization

    # Corner case q5 = pi
    print('R4_G_sym (q5=pi) =')
    pprint(simplify(R4_G_sym.subs({q5: pi}))) # just for visualization



    # Solve wrist rotation matrix

    # inspired by https://www.geometrictools.com/Documentation/EulerAngles.pdf
    if R4_G[2,2] < 1:

        if R4_G[2,2] > -1:
            # theta_5 between (0, pi)
            theta_5 = acos(R4_G[2,2])
            theta_4 = atan2(-R4_G[1,2], -R4_G[0,2])
            theta_6 = atan2(-R4_G[2,1], R4_G[2,0])

        else:
            # q5 = pi s.t. cos(q5) = -1 and sin(q5) = 0
            theta_5 = pi
            # Not a unique solution: q4 - q6 = atan2(-r10,-r00)
            theta_4 = S(0)
            theta_6 = - atan2(-R4_G[1,0], -R4_G[0,0])

    else:
        # q5 = 0 s.t. cos(q5) = 1 and sin(q5) = 0
        theta_5 = S(0)
        # Not a unique solution: q4 + q6 = atan2(r10, r00)
        theta_4 = S(0)
        theta_6 = atan2(R4_G[0,1], R4_G[0,0])

    # print('theta_4 =', theta_4)
    # print('theta_5 =', theta_5)
    # print('theta_6 =', theta_6)
    return theta_1, theta_2, theta_3, theta_4, theta_5, theta_6



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols



            # Joint angle symbols



            # Modified DH params



            # Define Modified DH Transformation matrix



            # Create individual transformation matrices



            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method

            # Get matrix form of position and orientation of end-effector
            O = get_pos_rot_mat(px, py, pz, roll, pitch, yaw)

            # Return from URDF to DH param coordinates (apply inverse correction)
            O1 = O * Transpose(RG_corr)

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = inverse_kinematics(O1)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
