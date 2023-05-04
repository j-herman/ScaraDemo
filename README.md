# ScaraDemo

To run the demo, download the code files into an existing catkin workspace.  You'll need to have ROS, Gazebo, and the appropriate gazebo-ros-control packages installed on your system.  

Compile using catkin_make from your_catkin_workspace/src.  To start the simulation:

`roslaunch hw3rbt_gazebo hw3rbt.launch `

The demo can be run with the command: 

`rosrun scara_demo scara_demo`

Note from 2023: inverseKinematics node is not starting automatically from the launch file.  If this happens, run in a separate terminal:

`rosrun fwd_kinematics inverseKinematics.py`
