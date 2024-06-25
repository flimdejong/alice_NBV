#! /usr/bin/env python3

# Connect arduino + power supply + camera

# Launch core:
# - Robot description
# - JointStatePublisher
# - ArduinoCommander
# - RobotStatePublisher
# - Moveit Fake Controllers/ move_group
# - Rviz
# - Realsense_camera

# Robot moves to starting position (straight up) 

# Robot moves to first position and takes depth cloud image
# Send Array to moveit. Wait for execution. Take depth cloud image.

# Example array: --> this example is a little like x_low
# [45, 44.57, 81.89, 3.53]

# Listen on move_completed. When returned true, get TF + depth cloud image

# computeTransform gets pose1 after returning True on move_completed
# rosToOpen3D gets PC from pose1 after returning True on move_completed

# Pose 1 PC needs to be preprocessed

# Robot moves to second position
# rosToOpen3D gets PC from pose1 after returning True on move_completed

# Pose 2 PC needs to be preprocessed

# The relative TF is computed and point clouds merged using the algorithms
# Pose2 also becomes Pose1 for the next iteration

# Repeat

# Python scripts necessary:
# - rosToOpen3D.py --> to convert realsense PC to Open3D format
# - moveit_execute_joint.py --> To listen for joint commands and send them to moveIt
# - ICP.py --> To carry out ICP merging
# - computeTransform --> To carry out transforms



# Moveit listens under node called execute_joint_movement, Float64MulitArray (must be send in radians!)
# Under move_completed, returns true or false for waiting until movement is done.
# preprocessing_status also returns True or false


# For x-axis send inverted commands.


# Pos 1
# x: 250, y: 0, z: 60
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.58913, 2.00016, 3.05527, 1.5708"]


# Pos 2
# x: 250, y: 0, z: 100
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.4634, 1.93729, 3.08679, 1.5708"]

# Pos 3
# x: 250, y: 0, z: 140
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.36787, 1.83942, 3.07075, 1.5108"]



