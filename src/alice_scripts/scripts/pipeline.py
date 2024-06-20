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



# Moveit listens under node called execute_joint_movement, Float64MulitArray (must be send in radians!)
# Under move_completed, returns true or false for waiting until movement is done.



# To do:
# - PC preprocessing
# - Merging of PC's



