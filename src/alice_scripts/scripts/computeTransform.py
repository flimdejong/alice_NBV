#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import time
import tf2_msgs
import tf.transformations as tr # For quartenion 
from sensor_msgs.msg import JointState

movement_complete = False
prev_joint_states = None

# Provide the transfrom from source to target.
def get_transform(source_frame, target_frame, tf_buffer):
    try:
        transform_stamped = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(1))

        # Get the translation coordinates
        translation = [transform_stamped.transform.translation.x,
                       transform_stamped.transform.translation.y,
                       transform_stamped.transform.translation.z]
        
        # Get the rotation quartenion values
        rotation = [transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w]
        
        return translation, rotation
    
    except:
        rospy.logerr(f"Failed to get transform from {source_frame} to {target_frame}")
        return None, None

def quaternion_to_rotation_matrix(quaternion):
    q0 = 0
    q1 = 0
    q2 = 0
    q3 = 0

    # Extract the qx, qy, qz, qw from the quartenion
    x, y, z, w = quaternion

    q0, q1, q2, q3 = x, y, z, w

    rot_matrix = np.array([
        [2*(q0**2 + q1**2) -1    , 2*(q1*q2 - q0*q3)      , 2*(q1*q3 + q0*q2)    ],
        [2*(q1*q2 + q0*q3)       , 2*(q0**2 + q2**2) -1   , 2*(q2*q3 - q0*q1)    ],
        [2*(q1*q3 - q0*q2)       , 2*(q2*q3 + q0*q1)      , 2*(q0**2 + q3**2) -1 ]
    ])

    return rot_matrix

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """

    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def joint_states_callback(msg):
    global movement_complete, prev_joint_states
    
    # Get the current joint states
    current_joint_states = msg.position
    
    # Check if there are previous joint states
    if prev_joint_states is not None:
        # Compare current joint states with previous joint states
        if np.allclose(current_joint_states, prev_joint_states):
            # If joint states are the same, movement is complete
            movement_complete = True
        else:
            # If joint states are different, movement is still in progress
            movement_complete = False
    
    # Update previous joint states with current joint states
    prev_joint_states = current_joint_states

def main():
    rospy.init_node('computeTransform')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage)

    rospy.loginfo("Finding TF for pose1")

    # Get the transformation matrix from world to pose1
    translation1, rotation1 = get_transform('world', 'camera_link', buffer)

    # Subscribe to the joint states topic
    joint_states_sub = rospy.Subscriber('/alice/joint_states', JointState, joint_states_callback)

    # Wait for the movement to finish
    rospy.loginfo("Waiting for movement to finish...")
    while not movement_complete:
        rospy.sleep(0.1)

    # Set execution_complete on False again.

    ####################
    ###### Pose 2 ######
    ####################

    # Clear buffer
    buffer.clear() 

    # Get the transformation matrix from world to pose2
    translation2, rotation2 = get_transform('world', 'camera_link', buffer)

    # Convert quaternions to rotation matrices
    transformation_matrix_1 = tr.quaternion_matrix(rotation1)
    transformation_matrix_2 = tr.quaternion_matrix(rotation2)

    # Fill last column and all rows
    transformation_matrix_1[:3, 3] = translation1
    transformation_matrix_2[:3, 3] = translation2

    # Compute the relative transformation from pose1 to pose2
    relative_transform = np.linalg.inv(transformation_matrix_1) @ transformation_matrix_2

    print("Transform 1: ")
    print("This should match tf_echo!!")
    print(transformation_matrix_1)

    print("Transform 2: ")
    print(transformation_matrix_2)

    # Print the relative transformation matrix
    print("Relative transformation matrix from source to target:")
    print(relative_transform)

    # Taking inverse to get TF from target to source
    print("target to source")
    print(np.linalg.inv(relative_transform))


if __name__ == '__main__':
    main()


