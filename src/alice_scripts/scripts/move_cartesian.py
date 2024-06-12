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

move_group.set_goal_tolerance(0.5)
move_group.set_num_planning_attempts(20)

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


#Set a pose goal
pose_goal = geometry_msgs.msg.Pose()

#Pose
pose_goal.orientation.x = 0.6066062028281202
pose_goal.orientation.y = 0.1630868366228961
pose_goal.orientation.z = 0.767120582212731
pose_goal.orientation.w = 0.13022139131643296

#Translation
pose_goal.position.x = -0.007045948116474077
pose_goal.position.y = 0.018825984527860093
pose_goal.position.z = 0.33320933951400644


# Plan and execute the motion
move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)

# Check if the planning was successful
if plan:
    print("Motion planning succeeded!")
else:
    print("Motion planning failed!")


# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()