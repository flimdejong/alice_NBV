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
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.36787, 1.83942, 3.07075, 1.5708"]




# Pos with 0
# x: 280, y: 0, z: 60
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.47085, 3.03042, 0.781914, 1.5708"]

# 280, 80
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.36279, 3.03171, 0.88869, 1.5708"]


# Pos with 0
# x: 270, y: 0, z: 110
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.15808, 3.10168, 1.02342, 1.5708"]


# Try with global registration only + full background. + don't use black == bad results







# newPos
# x: 210, y: 0, z: 280,
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 2.37478, 0.976217, 2.93219, 1.8"]

# newPos
# x: 210, y: 0, z: 210
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.872665, 1.50529, 3.09271, 1.68519, 1.5708"]

# newPos
# x: 160, y: 0, z: 240
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [0.785398, 1.1844, 3.08822, 2.01056, 1.5708"]





# newPos
# x: 50, y: 0, z: 250
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [1.0472, 0.588055, 3.05422, 2.64091, 1.8"]




# max 40 translation
# max 15 deg, 0.261799






## Most broad on top position.
# x: 30, y: 0, z: 300
# rostopic pub /execute_joint_movement std_msgs/Float64MultiArray "data: [1.5708, 0.984257, 2.19883, 3.1001, 1.8"]






# Purely transformations are not super effective, there is much overlap.
# Little overlap is the biggest issue. 

# So do we add full scene then or no?
# Pros: more 


# ICP work well with semi decent initial alignment + 



# Take picture one with the most amount possible to capture
# Then either filter the object out each time or at the end. 
# Tried it, failed. Only solution is to take very small PC's in sequence.



# Any registration also does not work. Try with just the object now.