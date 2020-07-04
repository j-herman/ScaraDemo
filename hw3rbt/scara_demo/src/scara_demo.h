#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <gazebo/gazebo.hh>
#include <vector>
#include <time.h>
#include <math.h>
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "std_msgs/Float64.h"
#include "fwd_kinematics/Joints.h"
#include "fwd_kinematics/InvPoseCalc.h"
#include "s_point/s_point.h"


const int JOINT_THREE_LOC = 5;
const int RANGE_XY = 149;
const int MIN_XY = 51;
const int RANGE_Z = 149;
const double TOLERANCE = 0.1;
