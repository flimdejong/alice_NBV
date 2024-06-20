#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

# The robot always defaults to this first
default_joint_state = JointState()
default_joint_state.name = ['base_joint', 'elbow_joint', 'shoulder_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
default_joint_state.position = [0, 1.5708, 1.5708, 1.5708, 1.5708] #Angle = 0, pointing straight up


# Function that is called to change joint_state
def moveit_callback(msg):
    global joint_state
    joint_state = msg


# joint_state_publisher is node that both sends messages to RobotState (which moveit and rviz use)
# and the arduinoCommander node, responsible for arduino-ros serial

def joint_state_publisher():
    global joint_state

    rospy.init_node('joint_state_publisher')

    # Publishes to robotState node (ROS ecosystem)
    pub_robotState = rospy.Publisher('/alice/joint_states', JointState, queue_size=10)

    # First time program runs, use default joint state
    joint_state = default_joint_state

    # Intialize prev_joint_state to the current joint_state
    prev_joint_state = joint_state

    joint_state.name = ['base_joint', 'elbow_joint', 'shoulder_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
    
    moveit_listener = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, moveit_callback, queue_size=10)

    # Publishes to the arduinoCommander node
    pub_arduinoCommander = rospy.Publisher('/alice/joint_states_arduino', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        # Convert to angles
        joint_angles_arduino = Float64MultiArray()
        joint_angles_arduino.data = [math.degrees(angle) for angle in joint_state.position]

        # Check and cap the joint angle in index [2], pos 3
        if joint_angles_arduino.data[2] < 30:
            joint_angles_arduino.data[2] = 30
            rospy.logerr("Joint angle of shoulder is lower than 30 degrees. Capping to 30 degrees.")
        elif joint_angles_arduino.data[2] > 150:
            joint_angles_arduino.data[2] = 150
            rospy.logerr("Joint angle of shoulder is higher than 150 degrees. Capping to 150 degrees.")

        # Publish the joint_state message
        joint_state.header.stamp = rospy.Time.now()
        pub_robotState.publish(joint_state)

        rospy.sleep(0.1)  # Sleep for 0.1 seconds to control the overall loop rate


if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass





