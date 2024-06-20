#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import time
import tf2_msgs
import tf.transformations as tr # For quartenion 
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

move_completed = False

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

def move_completed_cb(msg):
    global move_completed
    move_completed = msg.data
    rospy.loginfo("Move completed: %s", move_completed)

def main():
    global move_completed

    rospy.init_node('computeTransform')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Publisher for transformation matrix
    transform_pub = rospy.Publisher('/transformation_matrix', Float64MultiArray, queue_size=10)

    # Subscriber for move_completed topic
    move_completed_sub = rospy.Subscriber('/move_completed', Bool, move_completed_cb)


    ####################
    ###### Pose 1 ######
    ####################

    # Wait for first movement to finish
    rospy.loginfo("Waiting for move_completed to call...")

    # While move_completed is False, sleep. Once it turns True, code moves forward
    while not move_completed:
        rospy.sleep(0.1)

    # Set to false again
    move_completed = False
    
    rospy.loginfo("Waiting for TF")

    rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage)

    rospy.loginfo("Finding TF for pose1")

    # Get the transformation matrix from world to pose1
    translation1, rotation1 = get_transform('world', 'camera_link', buffer)

    rospy.loginfo("Received TF for pose1")

    while not rospy.is_shutdown():
        ####################
        ###### Pose 2 ######
        ####################

        # Wait for first movement to finish
        rospy.loginfo("Waiting for move_completed to call for next pose...")

        # While move_completed is False, sleep. Once it turns True, code moves forward
        while not move_completed:
            rospy.sleep(0.1)

        move_completed = False

        # Clear buffer
        buffer.clear() 

        rospy.loginfo("Waiting for TF")

        rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage)

        rospy.loginfo("Finding TF for pose2")

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

        # Taking inverse to get TF from target to source (pose2 to pose1), overload variable
        print("target to source")
        print(np.linalg.inv(relative_transform))

        # Create a Float64MultiArray message
        transform_msg = Float64MultiArray()
        transform_msg.layout.dim = [MultiArrayDimension('rows', 4, 4),
                                    MultiArrayDimension('cols', 4, 4)]
        transform_msg.data = relative_transform.flatten().tolist()

        # Publish the transformation matrix
        transform_pub.publish(transform_msg)

        # Make the source pose of the next iteration the pose of pose2 of the current one
        translation1 = translation2
        rotation1 = rotation2


if __name__ == '__main__':
    main()


