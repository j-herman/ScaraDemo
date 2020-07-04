#!/usr/bin/env python

import rospy
import tf
from math import atan2, sin, cos, sqrt, pow, pi
from std_msgs.msg import String
from fwd_kinematics.msg import *
from fwd_kinematics.srv import *

## This file provides a ROS service that calculates the joint values to position the Scara robot's 
## end effector as directed by the provided pose.  
## To initiate from the command line: rosrun fwd_kinematics inverseKinematics.py
## To call the service: 
## rosservice call /inverse_kinematics "inputPose: poseVector: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
## where poseVector is [x, y, z, roll, pitch, yaw]


def handle_inverse(data):
    pose = data.inputPose.poseVector

    # Set link length parameters as appropriate for a particular robot 
    d1 = 2.0
    a1 = 0.5
    a2 = 1.0

    d3 = d1 - pose[2]

#   If only (x,y,z) coordinates are provided, calculate joint values geometrically:
    if ((pose[3] == 0) and (pose[4] == 0) and (pose[5] == 0)):
      t2_cos = (pow(pose[0],2)+pow(pose[1],2) - 1.25)
      t2 = atan2(sqrt(1-pow(t2_cos,2)),t2_cos)
      t2 = round(t2,4)
      t1 = atan2(pose[1],pose[0]) - atan2(sin(t2),0.5+cos(t2))

      

#   If full pose is provided, calculate based on the Euler angles:
    else:
      r_mat = tf.transformations.euler_matrix(pose[3], pose[4], pose[5])
      t1_cos = pose[0] - r_mat[0][0]
      t1_sin = pose[1] - r_mat[1][0]
      t1 = atan2(t1_cos, t1_sin)
      t1 = atan2(t1_sin, t1_cos)
      t2 = atan2(r_mat[1][0], r_mat[0][0]) - t1


    output = Joints()
    output.qOne = t1
    output.qTwo = t2
    output.qThree = d3
    return output

def inverseKinematicsServer():
    rospy.init_node('inverseKinematics')
    s = rospy.Service('inverse_kinematics', InvPoseCalc, handle_inverse)
    print "Inverse Kinematics Calculator service ready"
    rospy.spin()

if __name__ == "__main__":
    inverseKinematicsServer()

