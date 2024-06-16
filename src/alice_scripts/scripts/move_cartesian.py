#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg

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
move_group.set_planner_id("RRTstar")
move_group.set_planning_time(10)

move_group.set_goal_position_tolerance(0.0001)
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

target_pose.position.x = 0.066
target_pose.position.y = 0.272
target_pose.position.z = 0.260
target_pose.orientation.x = 0
target_pose.orientation.y = 0
target_pose.orientation.z = 0
target_pose.orientation.w = 1

# Set the joint value target with approximate IK
move_group.set_joint_value_target(target_pose, 'camera_link', True)

# Plan and execute the motion
plan = move_group.go(wait=True)

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