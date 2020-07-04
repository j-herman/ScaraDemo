#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import String
from fwd_kinematics.msg import *

# On receipt of a Joints message, this function is called to calculate and display 
# the 4x4 transformation matrix for the full system

def calcKinematics(data):
    print("Calculating forward kinematics for:")
    print(data.qOne, data.qTwo, data.qThree)
    # Calculate needed trig functions 
    c1 = math.cos(data.qOne)
    s1 = math.sin(data.qOne)
    c2 = math.cos(data.qTwo)
    s2 = math.sin(data.qTwo)
    d3 = data.qThree

    # Set link length parameters as appropriate for a particular robot 
    a1 = 0.2
    a2 = 0.2
    print("Link lengths are set to:")
    print(a1, a2)

    # Create A matrices 

    A1 = np.array([[c1, -s1, 0, a1*c1], [s1, c1, 0, a1*s1], [0, 0, 1, 0], [0, 0, 0, 1]])
    A2 = np.array([[c2, s2, 0, a2*c2], [s2, -c2, 0, a2*s2], [0, 0, -1, 0], [0, 0, 0, 1]])
    A3 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d3], [0, 0, 0, 1]])
     
    A2_0 = np.matmul(A1,A2)
    T = np.matmul(A2_0,A3)

    print("Transformation matrix:")
    print(T)
    
    # Calculate Euler angles
    theta = phi = psi = 0
    if T[0,2] == 0 and T[1,2] == 0:
	if T[2,2] == 1:
	   theta = 0
           phi = 0
           psi = math.atan2(T[0,0], -T[0,1])
        else:
           theta = math.pi
           phi = 0
           psi = -math.atan2(-T[0,0], -T[0,1])
    else:
	theta = math.atan2(T[2,2], math.sqrt(1-T[2,2]*T[2,2]))
        phi = math.atan2(T[0,2], T[1,2])
        psi = math.atan2(-T[2,1], T[2,1])

    print(theta)
    print(phi)
    print(psi)
   
    # Output pose vector as array
    pose = Pose()
    pose.poseVector = np.concatenate((T[0:3,3], [theta, phi, psi]))
    print("Pose:")
    print(pose)

 
# On receipt of a Joints message via jacobianParams topic, this function is called to calculate the Jacobian

def calcJacobian(data):

    print("Calculating Jacobian for:")
    print(data.qOne, data.qTwo, data.qThree)

    # Calculate needed trig functions 
    c1 = math.cos(data.qOne)
    s1 = math.sin(data.qOne)
    c2 = math.cos(data.qTwo)
    s2 = math.sin(data.qTwo)
    d3 = data.qThree

    # Set link length parameters as appropriate for a particular robot 
    a1 = 0.2
    a2 = 0.2
    print("Link lengths are set to:")
    print(a1, a2)

    # Create A matrices 

    A1 = np.array([[c1, -s1, 0, a1*c1], [s1, c1, 0, a1*s1], [0, 0, 1, 0], [0, 0, 0, 1]])
    A2 = np.array([[c2, s2, 0, a2*c2], [s2, -c2, 0, a2*s2], [0, 0, -1, 0], [0, 0, 0, 1]])
    A3 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d3], [0, 0, 0, 1]])
     
    A2_0 = np.matmul(A1,A2)
    A3_0 = np.matmul(A2_0,A3)

    # Extract origins with respect to frame 0 and z axes with respect to frame i-1
    o1 = np.array([A1[0,3], A1[1,3], A1[2,3]])
    o2 = np.array([A2_0[0,3], A2_0[1,3], A2_0[2,3]])
    o3 = np.array([A3_0[0,3], A3_0[1,3], A3_0[2,3]])
    z0 = np.array([0, 0, 1]);
    z1 = np.array([A1[0,2], A1[1,2], A1[2,2]])
    z2 = np.array([A2[0,2], A2[1,2], A2[2,2]])

    # Create vectors for columns of Jacobian
    j1 = np.concatenate((np.cross(z0,o3), z0))
    j2 = np.concatenate((np.cross(z1,np.subtract(o3,o1)), z1))
    j3 = np.concatenate((z2,[0, 0, 0]))

    # Create Jacobian
    J = np.column_stack((j1,j2,j3))
    print("Jacobian:")
    print(J)


def kCalculator():

    # Create new uniquely named node to listen for "jointAngles" and "jacobianParams" topics
    rospy.init_node('kCalculator', anonymous=True)

    # When a Joints message is published to "jointAngles" topic, calls calcKinematics
    # Expects joint 1 and 2 values in radians, and joint 3 as a linear displacement
    # For testing, called from the command line using "rostopic pub -1 jointAngles fwd_kinematics/Joints Q1 Q2 Q3"
    # where Q1, Q2, Q3 are values for joints 1, 2, 3 of the SCARA robot described in the homework
    rospy.Subscriber("jointAngles", Joints, calcKinematics)

    # When a Joints message is published to "jacobianParams" topic, calls calcJacobian
    # Expects joint 1 and 2 values in radians, and joint 3 as a linear displacement
    # For testing, called from the command line using "rostopic pub -1 jacobianParams fwd_kinematics/Joints Q1 Q2 Q3"
    # where Q1, Q2, Q3 are values for joints 1, 2, 3 of the SCARA robot described in the homework
    rospy.Subscriber("jacobianParams", Joints, calcJacobian)

    # Keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    kCalculator()
