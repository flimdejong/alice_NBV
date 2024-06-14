#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def get_end_effector_position(joint_values):
    rospy.wait_for_service('compute_fk')
    try:
        compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        
        # Create a RobotState message
        robot_state = RobotState()
        robot_state.joint_state.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
        robot_state.joint_state.position = joint_values
        
        # Create a Header message
        header = Header()
        header.frame_id = 'base_link'

        fk_link_name = ['camera_link']
        
        # Call the service
        response = compute_fk(header, fk_link_name, robot_state)
        
        if response.error_code.val == response.error_code.SUCCESS:
            end_effector_pose = response.pose_stamped[0].pose
            end_effector_position = [end_effector_pose.position.x,
                                     end_effector_pose.position.y,
                                     end_effector_pose.position.z]
            return end_effector_position
        else:
            rospy.logerr("Failed to compute forward kinematics. Error code: %d" % response.error_code.val)
            return None
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

if __name__ == '__main__':
    rospy.init_node('end_effector_position_node')
    
    # Example joint values
    joint_values = [0.0, 0.5702, 1.5708, 0.9559, 1.5708]
    
    end_effector_position = get_end_effector_position(joint_values)
    
    if end_effector_position is not None:
        rospy.loginfo("End effector position: x=%.3f, y=%.3f, z=%.3f",
                      end_effector_position[0], end_effector_position[1], end_effector_position[2])