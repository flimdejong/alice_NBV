#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs
from std_msgs.msg import Float32MultiArray

def move_joint(msg):
    # Extract the joint angles from the message
    joint_angles = msg.data

    try:
        # Initialize the MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group_name = "arm"  # Replace with your robot's move group name
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame and end effector link
        move_group.set_pose_reference_frame("base_cyllinder")  # Replace with your robot's base frame
        move_group.set_end_effector_link("camera_link")  # Replace with your end effector link

        # Set the start state to the current state
        start_state = robot.get_current_state()
        move_group.set_start_state(start_state)

        # Set the joint angles for the arm
        move_group.set_joint_value_target(joint_angles)

        # Plan and execute the motion
        plan = move_group.go(wait=True)

        # Check if the planning and execution were successful
        if plan:
            rospy.loginfo("Motion planning and execution succeeded!")
        else:
            rospy.logerr("Motion planning and execution failed!")

    except Exception as e:
        rospy.logerr("Error occurred during motion planning and execution: {}".format(str(e)))

    finally:
        # Shutdown the MoveIt commander
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('move_end_effector', anonymous=True)

    # Create a subscriber to receive joint angles from the joint_pub topic
    rospy.Subscriber('joint_pub_rl', Float32MultiArray, move_joint)

    rospy.spin()