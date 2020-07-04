/* File: scara_demo.cpp
 * Author: Jessica Herman
 * License: NA
 *
 * This file demonstrates the functionality of the simplified Scara robot model implemented in the
 * set of packages titled "hw3_rbt".  It creates a ROS node to manage three joint command publishers
 * and two service clients that track joint and end effector positions.  A set of intermediate points
 * is generated in the task space of the robot.  Each is transformed into joint space via calls to
 * kinematics server nodes (previously implemented in Python).  Commands are published to PID position
 * controllers to move the robot sequentially through the point set.  The actual joint positions are
 * monitored to avoid sending overlapping commands.  As each joint position is achieved, the desired
 * task space position is checked against the Gazebo pose to validate the kinematics service.
 */

 #include "scara_demo.h"

// Random path generator of length num_nodes.  Returns the path.
// Much careful type conversion is needed to manage the transition from random
// integers to ROS message types.  The workspace estimate is rough and would
// need to be refined before using this code for anything more than a demo.
std::vector<geometry_msgs::Point> create_path(int num_nodes)
{
    std::vector<geometry_msgs::Point> path;
    for (int i=0;i<num_nodes;++i) {
        geometry_msgs::Point p;
        // Create a random point within the scara workspace
        int x = (rand() % (2*RANGE_XY)) - 149;
        double ymax = sqrt(pow(RANGE_XY,2) - pow(x,2));
        double ymin = sqrt(pow(MIN_XY,2) - pow(x,2));
        int y = rand() % static_cast<int>(ymax);
        while (y<ymin)
            y = rand() % static_cast<int>(ymax);
        int neg_y = rand() % 2;
        if (neg_y == 0)
            y = -y;
        int z = rand() % RANGE_Z + 50;
        p.x = static_cast<float>(x)/100.0;
        p.y = static_cast<float>(y)/100.0;
        p.z = static_cast<float>(z)/100.0;
        path.push_back(p);
    }
    return path;
}

// Loop through the path vector calling the inverse_kinematics service on each set of (x,y,z) coordinates.
// Create a vector of Joints objects to store the path in joint space coordinates.
std::vector<fwd_kinematics::Joints> inv_kinematics(ros::NodeHandle nh, std::vector<geometry_msgs::Point> path)
{
    std::vector<fwd_kinematics::Joints> j_path;
    ros::ServiceClient client = nh.serviceClient<fwd_kinematics::InvPoseCalc>("inverse_kinematics");
    fwd_kinematics::InvPoseCalc srv;
    std::vector<geometry_msgs::Point>::iterator it;
    for (it = path.begin(); it < path.end(); it++) {
        // Set (x,y,z) for the request; set Euler angle parameters to 0 to indicate pose-indifferent mode
        srv.request.inputPose.poseVector[0] = it->x;
        srv.request.inputPose.poseVector[1] = it->y;
        srv.request.inputPose.poseVector[2] = it->z;
        srv.request.inputPose.poseVector[3] = srv.request.inputPose.poseVector[4] = srv.request.inputPose.poseVector[5] = 0;
        if (client.call(srv)) {
            j_path.push_back(srv.response.outputJoints);
        }
        else {
            ROS_ERROR("Failed to call service ");
        }
    }
    return j_path;
}

// Loops for up to ten seconds, checking to see if all joints have reached their goal positions and exiting
// with success or timeout.
void move_scara(ros::ServiceClient client, std::vector<fwd_kinematics::Joints>::iterator it, ros::Publisher j1_pub, ros::Publisher j2_pub, ros::Publisher j3_pub)
{
    ros::Rate rate(1);

    gazebo_msgs::GetJointProperties srv;
    std_msgs::Float64 q1_temp;
    std_msgs::Float64 q2_temp;
    std_msgs::Float64 q3_temp;
    q1_temp.data = it->qOne;
    q2_temp.data = it->qTwo;
    q3_temp.data = -it->qThree;
    float joints[3];

    // Variables to check for joint values within tolerance and max time to wait
    bool done_1 = false;
    bool done_2 = false;
    bool done_3 = false;
    int i = 0;
    // Call gazebo joint state service and compare to desired value to see if all three joints are
    // at their goal positions
    while (!(done_1 && done_2 && done_3)) {
        j1_pub.publish(q1_temp);
        j2_pub.publish(q2_temp);
        j3_pub.publish(q3_temp);
        srv.request.joint_name = "base_to_link_one";
        if (client.call(srv)) {
            joints[0]=(float)srv.response.position[0];
            if (std::abs((float)q1_temp.data - joints[0]) < TOLERANCE)
                done_1 = true;
        }
        srv.request.joint_name = "link_one_to_link_two";
        if (client.call(srv)) {
            joints[1]=(float)srv.response.position[0];
            if (std::abs((float)q2_temp.data - joints[1]) < TOLERANCE)
                done_2 = true;
        }
        srv.request.joint_name = "link_two_to_link_three";
        if (client.call(srv)) {
            joints[2]=(float)srv.response.position[0];
            if (std::abs((float)q3_temp.data - joints[2]) < TOLERANCE)
                done_3 = true;
        }
        // Check 10 times, for a maximum of 10 seconds per position
        rate.sleep();
        if (i++ == 9) {
            // Report failure
            done_1 = true;
            done_2 = true;
            done_3 = true;
            std::cout<<"Never converged"<<std::endl;
        }
    }
}

// The main function: create the ROS node, set up publishers and service clients, and initiate the
// path-following sequence
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scara_demo");

    ros::NodeHandle nh;

    ros::Publisher j1_pub = nh.advertise<std_msgs::Float64>("/SCARA/joint1_position_controller/command", 5);
    ros::Publisher j2_pub = nh.advertise<std_msgs::Float64>("/SCARA/joint2_position_controller/command", 5);
    ros::Publisher j3_pub = nh.advertise<std_msgs::Float64>("/SCARA/joint3_position_controller/command", 5);
    ros::ServiceClient j_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    ros::ServiceClient link_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    srand (time(NULL));
    ros::Rate rate(10);

    // Create vectors for path in joint and task space to enable variable-length paths in later implementation
    // Generate a random path within the Scara workspace of length path_length.

    int path_length = 3;
    std::vector<geometry_msgs::Point> path = create_path(path_length);
    std::vector<geometry_msgs::Point>::iterator it = path.begin();

    // Convert path to joint space via inverse kinematics service from package fwd_kinematics
    std::vector<fwd_kinematics::Joints> j_path = inv_kinematics(nh, path);
    std::vector<fwd_kinematics::Joints>::iterator it2;

    gazebo_msgs::GetLinkState srv;
    srv.request.reference_frame = "SCARA::base_link";

    // Iterate through generated path, moving robot and generating position observations for comparison
    for (it2 = j_path.begin(); it2 < j_path.end(); it2++) {
        SPoint pos(*it++);
        std::cout<<"Moving to: "<<pos<<std::endl;
        move_scara(j_client, it2, j1_pub, j2_pub, j3_pub);
        srv.request.link_name = "SCARA::link_three";
        if (link_client.call(srv)) {
            SPoint fpos(srv.response.link_state.pose.position);
            std::cout<<"Final position: "<<fpos<<std::endl;
            std::cout<<"Linear distance (error): "<<std::setprecision(4)<<fpos.point_compare(pos)<<std::endl;
        }
        else {
            ROS_ERROR("Failed to call service ");
        }
        rate.sleep();
    }

    // Spin to keep node information up for troubleshooting; Ctrl-C to quit
    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
}
