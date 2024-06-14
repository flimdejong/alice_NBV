#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import time

def get_transform(source_frame, target_frame, tf_buffer):
    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())

        # Get the translation coordinates
        translation = [transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z]
        
        # Get the rotation quartenion values
        rotation = [transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
        
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


def main():
    rospy.init_node('computeTransform')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Wait 1 sec
    rospy.sleep(1.0)

    rospy.loginfo("Finding TF for pose1")

    # Get the transformation matrix from world to pose1
    translation1, rotation1 = get_transform('world', 'camera_link', buffer)


    # Wait for 10 seconds and execute movement
    countdown_time = 10
    rospy.loginfo(f"Countdown started. Movement will execute in {countdown_time} seconds...")

    while countdown_time > 0:
        rospy.loginfo(f"Time remaining: {countdown_time} seconds")
        time.sleep(1.0)  # Delay for 1 second
        countdown_time -= 1

    rospy.loginfo("Countdown finished. Executing movement...")


    # Get the transformation matrix from world to pose2
    translation2, rotation2 = get_transform('world', 'camera_link', buffer)


    # Convert quaternions to rotation matrices
    rotation_matrix1 = quaternion_to_rotation_matrix(rotation1)
    rotation_matrix2 = quaternion_to_rotation_matrix(rotation2)

    # Construct the transformation matrices
    transformation_matrix1 = np.eye(4) # Create matrix placeholder

    #Fill the most left 3x3 with the rotation matrix
    transformation_matrix1[:3, :3] = rotation_matrix1

    #Fill first three rows, third column with the translation matrices
    transformation_matrix1[:3, 3] = translation1

    # Same here
    transformation_matrix2 = np.eye(4) # Create matrix placeholder
    transformation_matrix2[:3, :3] = rotation_matrix2
    transformation_matrix2[:3, 3] = translation2

    # Compute the relative transformation from pose1 to pose2
    relative_transform = np.linalg.inv(transformation_matrix1) @ transformation_matrix2

    # Print the relative transformation matrix
    print("Relative transformation matrix from pose1 to pose2:")
    print(relative_transform)

if __name__ == '__main__':
    main()

