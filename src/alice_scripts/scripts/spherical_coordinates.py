import sys
import rospy
import moveit_commander
import math
import geometry_msgs.msg

def spherical_to_cartesian(r, theta, phi):
    x = r * math.sin(phi) * math.cos(theta)
    y = r * math.sin(phi) * math.sin(theta)
    z = r * math.cos(phi)
    return x, y, z

def move_end_effector(r, theta, phi):
    # Initialize the MoveIt commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"  # Replace with your robot's move group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the reference frame and end effector link
    move_group.set_pose_reference_frame("base_cyllinder")
    move_group.set_end_effector_link("camera_link")

    # Convert spherical coordinates to Cartesian coordinates
    x, y, z = spherical_to_cartesian(r, theta, phi)

    # Create a target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.w = 1.0

    # Set the target pose
    move_group.set_pose_target(target_pose)

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

    # Specify the desired spherical coordinates
    r = 0.5  # Radial distance
    theta = 0.4  # Azimuthal angle (in radians)
    phi = 1.57  # Polar angle (in radians)

    move_end_effector(r, theta, phi)