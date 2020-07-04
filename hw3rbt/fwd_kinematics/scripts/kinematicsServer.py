#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf
from std_msgs.msg import String
from fwd_kinematics.msg import *
from fwd_kinematics.srv import *

def handle_kinematics(data):
    print("Calculating forward kinematics for:")
    print(data.inputJoints.qOne, data.inputJoints.qTwo, data.inputJoints.qThree)
    # Calculate needed trig functions 
    c1 = math.cos(data.inputJoints.qOne)
    s1 = math.sin(data.inputJoints.qOne)
    c2 = math.cos(data.inputJoints.qTwo)
    s2 = math.sin(data.inputJoints.qTwo)
    d3 = data.inputJoints.qThree

    # Set link length parameters as appropriate for a particular robot 
    d1 = 2.0
    a1 = 0.5
    a2 = 1.0

    # Create A matrices 

    A1 = np.array([[c1, -s1, 0, a1*c1], [s1, c1, 0, a1*s1], [0, 0, 1, d1], [0, 0, 0, 1]])
    A2 = np.array([[c2, s2, 0, a2*c2], [s2, -c2, 0, a2*s2], [0, 0, -1, 0], [0, 0, 0, 1]])
    A3 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d3], [0, 0, 0, 1]])

    A2_0 = np.matmul(A1,A2)
    T = np.matmul(A2_0,A3)

    # Calculate Euler angles
    theta = phi = psi = 0
    temp = tf.transformations.euler_from_matrix(T)
    theta = temp[0];
    phi = temp[1];
    psi = temp[2];   
    # Output pose vector as array
    pose = Pose()
    pose.poseVector = np.concatenate((T[0:3,3], [theta, phi, psi]))
    return pose

def kinematicsServer():
    rospy.init_node('kinematicsServer')
    s = rospy.Service('forward_kinematics', PoseCalc, handle_kinematics)
    print "Kinematics Calculator service ready"
    rospy.spin()

if __name__ == "__main__":
    kinematicsServer()

