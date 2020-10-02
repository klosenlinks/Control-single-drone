## Scope of the project
This is a simple simulation of a ROS-enabled UAV position controller. It is availaible standalone using the provided dynamics for the robot, or using Gazebo for the simulation. The files are also provided to control a real drone, however they are hardware-specific.

## Controllers
The two controllers available are a PID-based one and a Sliding Mode controller. The PID-based is actually a succession of two controllers : the first one is a PID used to obtain the desired force to be applied to the drone, which is used a) to compute the desired thrust, and b) as an input to the second controller which is a PD working on the quaternion orientation to obtain the desired torques.
The Sliding Mode controller has the same structure, with the first PID being the same but using a sliding mode control for orientation.

## Standalone simulation
First run the node "dynamic_model" to start the simulation. You can now enable the PID control of the drone by running the node "PID_control", or the sliding mode control with "SM_control". The desired pose should be published on /desired_pose in both cases.

## Gazebo simulation
Launchfiles are provided for the gazebo simulation, in order to use either PID or SM based control, and simulating with or without a wind-like perturbation

## Trajectory
As stated, the desired pose must be published on "/desired_pose". However there is a node "trajectory" provided that already wraps the pose publishing into a node, using an upward helicoidal trajectory as an example.
