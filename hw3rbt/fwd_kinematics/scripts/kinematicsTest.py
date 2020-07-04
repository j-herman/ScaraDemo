#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from fwd_kinematics.msg import *
from fwd_kinematics.srv import *

## Test utility that takes a set of joint values as arguments, calls
## the forward_kinematics service to get a task-space pose, then feeds
## that pose into the inverse_kinematics service.  The returned values
## should match the initial arguments.
## Call with: rosrun fwd_kinematics kinematicsTest.py 0.0 0.0 0.0 after
## bringing up the forward and inverse kinematics services, or start all
## three from the kinem.launch file:
## roslaunch fwd_kinematics kinem.launch a:=0.0 b:=0.0 c:=0.0



def kinematics_client(j1, j2, j3):
    rospy.wait_for_service('forward_kinematics')
    try:
        fwd_service = rospy.ServiceProxy('forward_kinematics', PoseCalc)
        jointsIn = Joints()
        jointsIn.qOne = j1
        jointsIn.qTwo = j2
        jointsIn.qThree = j3
        poseOut = fwd_service(jointsIn)
        print("Found pose:")
        print(poseOut)
        inv_service = rospy.ServiceProxy('inverse_kinematics', InvPoseCalc)
        finalJoints = inv_service(poseOut.outputPose)
        print("Found joints:")
        print(finalJoints)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    if len(sys.argv) >= 4:
        j1 = float(sys.argv[1])
        j2 = float(sys.argv[2])
        j3 = float(sys.argv[3])
    else:
        print("Bad arguments")
	print(sys.argv)
        sys.exit(1)
    print("Requesting kinematics")
    kinematics_client(j1, j2, j3)


