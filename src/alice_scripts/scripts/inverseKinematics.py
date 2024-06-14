#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def get_joint_values(end_effector_pose):
    rospy.wait_for_service('compute_ik')
    try:
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        
        # Create a PositionIKRequest message
        ik_request = PositionIKRequest()
        ik_request.group_name = 'arm'
        ik_request.robot_state = RobotState()
        ik_request.robot_state.joint_state.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = 'base_link'
        ik_request.pose_stamped.pose = end_effector_pose
        
        # Call the service
        response = compute_ik(ik_request)
        
        if response.error_code.val == response.error_code.SUCCESS:
            joint_values = response.solution.joint_state.position
            return joint_values
        else:
            rospy.logerr("Failed to compute inverse kinematics.")
            return None
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

if __name__ == '__main__':
    rospy.init_node('joint_values_node')

    # Initialize MoveGroupCommander
    group_name = 'arm'
    move_group = MoveGroupCommander(group_name)
    
    # Get the name of the end effector link
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo("End effector link: %s", end_effector_link)
    
    # Example end effector pose
    end_effector_pose = PoseStamped()
    end_effector_pose.header.frame_id = 'base_link'
    end_effector_pose.pose.position.x = 0.227
    end_effector_pose.pose.position.y = 0.127
    end_effector_pose.pose.position.z = 0.197
    end_effector_pose.pose.orientation.x = 0.0
    end_effector_pose.pose.orientation.y = 0.0
    end_effector_pose.pose.orientation.z = 0.0
    end_effector_pose.pose.orientation.w = 1.0
    
    joint_values = get_joint_values(end_effector_pose.pose)
    
    if joint_values is not None:
        rospy.loginfo("Joint values: %s", joint_values)