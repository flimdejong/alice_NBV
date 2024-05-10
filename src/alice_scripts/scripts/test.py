#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    # Get the current joint positions from the message
    joint_positions = msg.position

    # Create a MoveIt commander
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a RobotState object
    robot_state = robot.get_current_state()

    # Set the joint positions in the RobotState object
    joint_names = robot_state.joint_state.name
    robot_state.joint_state.position = joint_positions

    # Set the start state for planning
    move_group.set_start_state(robot_state)

    # Plan and execute the motion (example)
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.z -= 0.1  # Example: Move the end effector down by 0.1 meters
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def main():
    rospy.init_node('moveit_set_start_state', anonymous=True)

    # Subscribe to the joint states topic
    rospy.Subscriber('/alice/joint_states', JointState, joint_state_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass