#! /usr/bin/env python

import rospy
import sys
import moveit_commander

def move_end_effector(target_joint_angles):

    # Initialize the MoveIt commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"  # Replace with your robot's move group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the reference frame and end effector link
    move_group.set_pose_reference_frame("base_cyllinder")  # Replace with your robot's base frame
    move_group.set_end_effector_link("camera_link")  # Replace with your end effector link

    # Set the joint angles for the arm
    move_group.set_joint_value_target(target_joint_angles)

    # Plan and execute the motion
    plan = move_group.go(wait=True)

    # Check if the planning and execution were successful
    if plan:
        rospy.loginfo("Motion planning and execution succeeded!")
    else:
        rospy.logerr("Motion planning and execution failed!")

    # Shutdown the MoveIt commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('move_end_effector', anonymous=True)

    # Specify the desired joint angles (in radians)
    joint_angles = [0.6, 1.8, 1.4, 0.05]  # Replace with your desired joint angles

    move_end_effector(joint_angles)