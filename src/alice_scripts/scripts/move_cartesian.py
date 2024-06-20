#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np

# remap again
joint_state_topic = ['joint_states:=/alice/joint_states']

# Initialize the MoveIt commander
moveit_commander.roscpp_initialize(joint_state_topic)

#Initialize node
rospy.init_node('move_cartesian', anonymous=True)

#Create a robotcommander instance
robot = moveit_commander.RobotCommander()

#Initiate planning scene
scene = moveit_commander.PlanningSceneInterface()

#Set group to move
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_planning_time(5)


##########################
##### Set Tolerances #####
##########################

move_group.set_goal_position_tolerance(0.01)
move_group.set_goal_orientation_tolerance(np.deg2rad(20))
move_group.set_num_planning_attempts(10)

#################################
##### Get robot information #####
#################################

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)


print("Printing joint positions: ")
print (move_group.get_current_pose())

# Define the target pose
target_pose = geometry_msgs.msg.Pose()
# target_pose.position.x = 0.264
# target_pose.position.y = 0.066
# target_pose.position.z = 0.234
# target_pose.orientation.x = 0.707
# target_pose.orientation.y = 0.707
# target_pose.orientation.z = 0.0
# target_pose.orientation.w = 0

target_pose.position.x = 0.26
target_pose.position.y = 0.06
target_pose.position.z = 0.12
target_pose.orientation.x = 3.094
target_pose.orientation.y = 0
target_pose.orientation.z = 1.567

# Set the joint value target with approximate IK
move_group.set_joint_value_target(target_pose, 'camera_link', True)

# Plan and execute the motion
plan = move_group.plan()

# Check if planning was successful
if plan[0]:
    # Get the planned trajectory
    planned_trajectory = plan[1]
    
    # Get the last point in the planned trajectory (end effector pose)
    end_effector_pose = move_group.get_current_pose().pose
    
    # Get the desired end effector pose from the target pose
    desired_pose = target_pose  # Assuming target_pose is the desired end effector pose
    
    # Get the current end effector pose
    current_pose = move_group.get_current_pose().pose
    
    # Calculate the position and orientation differences between the planned and desired poses
    position_diff_desired = (end_effector_pose.pose.position.x - desired_pose.pose.position.x,
                             end_effector_pose.pose.position.y - desired_pose.pose.position.y,
                             end_effector_pose.pose.position.z - desired_pose.pose.position.z)
    
    orientation_diff_desired = (end_effector_pose.pose.orientation.x - desired_pose.pose.orientation.x,
                                end_effector_pose.pose.orientation.y - desired_pose.pose.orientation.y,
                                end_effector_pose.pose.orientation.z - desired_pose.pose.orientation.z,
                                end_effector_pose.pose.orientation.w - desired_pose.pose.orientation.w)
    
    # Calculate the position and orientation differences between the planned and current poses
    position_diff_current = (end_effector_pose.pose.position.x - current_pose.pose.position.x,
                             end_effector_pose.pose.position.y - current_pose.pose.position.y,
                             end_effector_pose.pose.position.z - current_pose.pose.position.z)
    
    orientation_diff_current = (end_effector_pose.pose.orientation.x - current_pose.pose.orientation.x,
                                end_effector_pose.pose.orientation.y - current_pose.pose.orientation.y,
                                end_effector_pose.pose.orientation.z - current_pose.pose.orientation.z,
                                end_effector_pose.pose.orientation.w - current_pose.pose.orientation.w)
    
    # Set the tolerance thresholds for position and orientation
    position_tolerance = 0.01  # Adjust this value based on your requirements
    orientation_tolerance = 0.01  # Adjust this value based on your requirements
    
    # Set the minimum movement thresholds for position and orientation
    min_position_movement = 0.001  # Adjust this value based on your requirements
    min_orientation_movement = 0.001  # Adjust this value based on your requirements
    
    # Check if the position and orientation differences exceed the tolerance thresholds
    if any(abs(diff) > position_tolerance for diff in position_diff_desired) or \
       any(abs(diff) > orientation_tolerance for diff in orientation_diff_desired):
        print("Planned end effector pose deviates too far from the desired pose.")
        # Disrupt the execution or take appropriate action
    elif all(abs(diff) < min_position_movement for diff in position_diff_current) and \
         all(abs(diff) < min_orientation_movement for diff in orientation_diff_current):
        print("Planned end effector pose is too close to the current pose. No movement needed.")
    else:
        # If the planned end effector pose is acceptable, execute the motion
        move_group.execute(planned_trajectory, wait=True)
else:
    print("Motion planning failed.")



# Check if the planning and execution was successful
if plan:
    print("Motion planning and execution succeeded!")
else:
    print("Motion planning and execution failed!")

# Stop any residual movement
move_group.stop()

# Clear the targets
move_group.clear_pose_targets()
#move_group.clear_joint_value_targets()