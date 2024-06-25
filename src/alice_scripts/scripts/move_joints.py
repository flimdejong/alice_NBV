#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from moveit_commander.exception import MoveItCommanderException

################
"""
I don't know why but for some reason when inputting commands to this topic,
send them as base_joint, shoulder_joint, elbow_joint, etc. Do use the correct format.
"""

def move_end_effector_cb(target_joint_angles):

    target_joint_angles_list = list(target_joint_angles.data)

    rospy.loginfo("move_joints: Planning a path!")

    # Set the joint angles for the arm
    move_group.set_joint_value_target(target_joint_angles_list)

    # Plan and execute the motion
    plan = move_group.go(wait=True)

    # Check if the planning and execution were successful
    if plan:
        rospy.loginfo("move_joints: Motion planning and execution succeeded!")
        rospy.loginfo("move_joints: Waiting 5 seconds to give the physical robot some time to move")
        rospy.sleep(5)
        rospy.logwarn("move_joints: Done, sending move_completed = True")

        move_completed_pub.publish(True)
    else:
        rospy.logerr("Motion planning and execution failed!")
        move_completed_pub.publish(False)

        
    move_group.clear_pose_targets()


# Initialize the MoveIt commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame and end effector link
move_group.set_pose_reference_frame("base_cyllinder")
move_group.set_end_effector_link("camera_link")

move_group.set_planning_time(0.1)

rospy.init_node('move_joints', anonymous=True)

move_end_effector = rospy.Subscriber('execute_joint_movement',Float64MultiArray, move_end_effector_cb)

# Publisher for sending a message when movement is completed
move_completed_pub = rospy.Publisher('move_completed', Bool, queue_size=10)

rospy.spin()